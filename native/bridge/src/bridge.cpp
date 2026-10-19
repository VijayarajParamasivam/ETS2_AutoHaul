// autohaul_bridge.dll: the only AutoHaul code that runs inside the game process.
//
// * Telemetry SDK -> shared memory "Local\AutoHaul_Telemetry" (header, seqlock ring of frames,
//   configuration block, gameplay event ring) and a named event pulsed on every frame.
// * Input SDK semantical device "autohaul" <- shared memory "Local\AutoHaul_Controls".
// * Fail-safes that do not depend on the Python side being healthy:
//     - watchdog: no fresh heartbeat (default 250 ms) -> every input released
//     - kill keys polled each input frame -> every input released, latched until re-armed
//     - game paused -> every input released
//
// All SDK callbacks run on the game's main thread and must return quickly: no locks, no I/O
// apart from rare log lines. Layout and channel tables come from generated/autohaul_layout.h.

#include <windows.h>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include "scssdk_input.h"
#include "scssdk_input_device.h"
#include "scssdk_input_event.h"
#include "scssdk_telemetry.h"
#include "common/scssdk_telemetry_common_configs.h"
#include "common/scssdk_telemetry_common_gameplay_events.h"
#include "eurotrucks2/scssdk_eut2.h"
#include "eurotrucks2/scssdk_input_eut2.h"
#include "eurotrucks2/scssdk_telemetry_eut2.h"

#include "generated/autohaul_layout.h"

using namespace autohaul;

namespace {

constexpr char kBridgeVersion[] = "0.1.0";
constexpr uint32_t kDefaultWatchdogMs = 250;
constexpr uint32_t kMinWatchdogMs = 50;
constexpr uint32_t kMaxWatchdogMs = 1000;
constexpr uint32_t kMaxPendingPulses = 8;
const int kDefaultKillVk[] = {VK_OEM_5 /* backslash */, VK_PAUSE};

static_assert(offsetof(TelemetryFrame, seq_begin) == 0, "seq_begin must be first");
static_assert(offsetof(TelemetryFrame, seq_end) == sizeof(TelemetryFrame) - 8, "seq_end must be last");

struct Mapping {
    HANDLE handle = nullptr;
    uint8_t* view = nullptr;
};

struct InputState {
    float axes[NUM_AXES] = {};
    bool buttons[NUM_BUTTONS] = {};
};

struct Bridge {
    // shared memory
    Mapping tel;
    Mapping ctl;
    HANDLE frame_event = nullptr;
    TelemetryHeader* hdr = nullptr;
    TelemetryFrame* frames = nullptr;
    ConfigBlock* cfg = nullptr;
    EventSlot* events = nullptr;
    ControlsBlock* controls = nullptr;
    uint64_t qpc_freq = 1;

    // telemetry
    scs_log_t log = nullptr;
    bool telemetry_on = false;
    TelemetryFrame work{};
    uint64_t frame_seq = 0;
    uint64_t event_seq = 0;
    bool paused = true;
    uint32_t channels_ok = 0;
    std::unordered_map<std::string, std::vector<uint32_t>> attr_lookup;  // "<section>|<name>" -> attr indices

    // input
    bool input_on = false;
    scs_input_device_input_t input_defs[NUM_INPUTS] = {};
    scs_input_event_t queue[NUM_INPUTS] = {};
    uint32_t queue_len = 0;
    uint32_t queue_pos = 0;
    InputState sent;                 // last values the game actually fetched
    bool force[NUM_INPUTS] = {};     // re-send even if unchanged (after (re)activation)
    uint32_t rr_start = 0;           // queue starts just after the last delivered input (fairness)
    bool client_known = false;
    uint32_t client_pid = 0;
    uint32_t seen_pulses[NUM_BUTTONS] = {};
    uint32_t pending_pulses[NUM_BUTTONS] = {};
    uint8_t pulse_phase[NUM_BUTTONS] = {};  // 0 idle, 1 pressed this frame, 2 released this frame
    bool kill_latched = false;
    uint32_t last_rearm = 0;
    bool was_alive = false;
    uint32_t log_budget = 50;        // rate-limit runtime log lines
};

Bridge g;
std::atomic<int> g_test_kill{0};

void logf(scs_log_type_t type, const char* fmt, ...) {
    if (!g.log) {
        return;
    }
    char buf[512];
    int n = std::snprintf(buf, sizeof buf, "[autohaul] ");
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf + n, sizeof buf - n, fmt, args);
    va_end(args);
    g.log(type, buf);
}

void log_runtime(const char* fmt, const char* arg) {
    if (g.log_budget == 0) {
        return;
    }
    if (--g.log_budget == 0) {
        logf(SCS_LOG_TYPE_warning, "runtime log budget exhausted; further messages suppressed");
        return;
    }
    logf(SCS_LOG_TYPE_message, fmt, arg);
}

uint64_t qpc_now() {
    LARGE_INTEGER v;
    QueryPerformanceCounter(&v);
    return static_cast<uint64_t>(v.QuadPart);
}

std::wstring object_name(const wchar_t* base) {
    // AUTOHAUL_SHM_SUFFIX isolates test runs (fake_game) from a running game.
    std::wstring name(base);
    wchar_t suffix[64];
    DWORD n = GetEnvironmentVariableW(L"AUTOHAUL_SHM_SUFFIX", suffix, 64);
    if (n > 0 && n < 64) {
        name += suffix;
    }
    return name;
}

bool open_mapping(Mapping& m, const wchar_t* base, uint32_t size) {
    if (m.view) {
        return true;
    }
    std::wstring name = object_name(base);
    m.handle = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, size, name.c_str());
    if (!m.handle) {
        return false;
    }
    m.view = static_cast<uint8_t*>(MapViewOfFile(m.handle, FILE_MAP_ALL_ACCESS, 0, 0, size));
    if (!m.view) {
        CloseHandle(m.handle);
        m.handle = nullptr;
        return false;
    }
    return true;
}

void close_mapping(Mapping& m) {
    if (m.view) {
        UnmapViewOfFile(m.view);
    }
    if (m.handle) {
        CloseHandle(m.handle);
    }
    m = Mapping{};
}

void copy_str(char* dst, size_t cap, const char* src) {
    if (!src) {
        src = "";
    }
    const size_t n = strnlen(src, cap - 1);
    std::memcpy(dst, src, n);
    dst[n] = '\0';
}

void set_flag(uint32_t flag, bool on) {
    if (!g.hdr) {
        return;
    }
    uint32_t f = g.hdr->bridge_flags;
    g.hdr->bridge_flags = on ? (f | flag) : (f & ~flag);
}

bool ensure_shared_memory() {
    if (g.hdr) {
        return true;
    }
    if (!open_mapping(g.tel, kTelemetryMapping, kTelemetrySize) || !open_mapping(g.ctl, kControlsMapping, kControlsSize)) {
        logf(SCS_LOG_TYPE_error, "cannot create shared memory (error %lu)", GetLastError());
        close_mapping(g.tel);
        close_mapping(g.ctl);
        return false;
    }
    std::wstring ev = object_name(kFrameEvent);
    g.frame_event = CreateEventW(nullptr, FALSE, FALSE, ev.c_str());

    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    g.qpc_freq = static_cast<uint64_t>(f.QuadPart);

    g.hdr = reinterpret_cast<TelemetryHeader*>(g.tel.view);
    g.frames = reinterpret_cast<TelemetryFrame*>(g.tel.view + kFramesOffset);
    g.cfg = reinterpret_cast<ConfigBlock*>(g.tel.view + kConfigOffset);
    g.events = reinterpret_cast<EventSlot*>(g.tel.view + kEventsOffset);
    g.controls = reinterpret_cast<ControlsBlock*>(g.ctl.view);
    // Presses counted before this DLL instance existed (e.g. by a client that died before an
    // `sdk reload`) must not fire now: start from whatever the block already holds.
    std::memcpy(g.seen_pulses, g.controls->pulses, sizeof g.seen_pulses);

    // (Re)initialise the telemetry mapping. Readers notice frame_counter going backwards and resync.
    std::memset(g.tel.view, 0, kTelemetrySize);
    TelemetryHeader& h = *g.hdr;
    h.layout_version = kLayoutVersion;
    h.layout_hash = kLayoutHash;
    h.header_size = sizeof(TelemetryHeader);
    h.frame_size = sizeof(TelemetryFrame);
    h.ring_slots = RING_SLOTS;
    h.frames_offset = kFramesOffset;
    h.config_offset = kConfigOffset;
    h.config_size = sizeof(ConfigBlock);
    h.events_offset = kEventsOffset;
    h.event_size = sizeof(EventSlot);
    h.event_slots = EVENT_SLOTS;
    h.game_pid = GetCurrentProcessId();
    h.qpc_frequency = g.qpc_freq;
    h.watchdog_ms = kDefaultWatchdogMs;
    h.input_count = NUM_INPUTS;
    h.paused = 1;
    copy_str(h.bridge_version, sizeof h.bridge_version, kBridgeVersion);
    std::atomic_thread_fence(std::memory_order_release);
    h.magic = kTelemetryMagic;  // last: readers wait for the magic
    return true;
}

// ------------------------------------------------------------------ telemetry: values

void store_value(uint8_t* base, uint32_t offset, ValueType type, uint32_t size, const scs_value_t& v) {
    uint8_t* dst = base + offset;
    switch (type) {
        case ValueType::Bool: *dst = v.value_bool.value ? 1 : 0; break;
        case ValueType::S32: std::memcpy(dst, &v.value_s32.value, 4); break;
        case ValueType::U32: std::memcpy(dst, &v.value_u32.value, 4); break;
        case ValueType::U64: std::memcpy(dst, &v.value_u64.value, 8); break;
        case ValueType::Float: std::memcpy(dst, &v.value_float.value, 4); break;
        case ValueType::Double: std::memcpy(dst, &v.value_double.value, 8); break;
        case ValueType::FVector: {
            Vec3 out{v.value_fvector.x, v.value_fvector.y, v.value_fvector.z};
            std::memcpy(dst, &out, sizeof out);
            break;
        }
        case ValueType::FPlacement: {
            const auto& p = v.value_fplacement;
            FPlacement out{p.position.x, p.position.y, p.position.z, p.orientation.heading, p.orientation.pitch, p.orientation.roll};
            std::memcpy(dst, &out, sizeof out);
            break;
        }
        case ValueType::DPlacement: {
            const auto& p = v.value_dplacement;
            DPlacement out{p.position.x, p.position.y, p.position.z, p.orientation.heading, p.orientation.pitch, p.orientation.roll, 0};
            std::memcpy(dst, &out, sizeof out);
            break;
        }
        case ValueType::String: copy_str(reinterpret_cast<char*>(dst), size, v.value_string.value); break;
    }
}

SCSAPI_VOID on_channel(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t context) {
    if (!value) {
        return;
    }
    const uint32_t id = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(context));
    const ChannelDef& def = kChannels[id];
    store_value(reinterpret_cast<uint8_t*>(&g.work), def.offset, def.type, def.size, *value);
    g.work.channel_seen[id / 64] |= (1ull << (id % 64));
}

// ------------------------------------------------------------------ telemetry: events

SCSAPI_VOID on_frame_start(const scs_event_t, const void* const info, const scs_context_t) {
    const auto* fs = static_cast<const scs_telemetry_frame_start_t*>(info);
    g.work.frame_flags = fs->flags;
    g.work.render_time_us = fs->render_time;
    g.work.simulation_time_us = fs->simulation_time;
    g.work.paused_simulation_time_us = fs->paused_simulation_time;
}

SCSAPI_VOID on_frame_end(const scs_event_t, const void* const, const scs_context_t) {
    if (!g.hdr) {
        return;
    }
    const uint64_t n = ++g.frame_seq;
    g.work.frame_index = n;
    g.work.publish_qpc = qpc_now();
    g.work.paused = g.paused ? 1 : 0;

    TelemetryFrame* slot = &g.frames[(n - 1) % RING_SLOTS];
    volatile uint64_t* begin = &slot->seq_begin;
    volatile uint64_t* end = &slot->seq_end;
    *begin = n;
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(reinterpret_cast<uint8_t*>(slot) + 8, reinterpret_cast<const uint8_t*>(&g.work) + 8, sizeof(TelemetryFrame) - 16);
    std::atomic_thread_fence(std::memory_order_release);
    *end = n;
    std::atomic_thread_fence(std::memory_order_release);
    g.hdr->frame_counter = n;
    if (g.frame_event) {
        SetEvent(g.frame_event);
    }
}

SCSAPI_VOID on_pause_change(const scs_event_t event, const void* const, const scs_context_t) {
    g.paused = (event == SCS_TELEMETRY_EVENT_paused);
    if (g.hdr) {
        g.hdr->paused = g.paused ? 1 : 0;
    }
}

int find_section(const char* id) {
    if (!id) {
        return -1;
    }
    const char* key = std::strcmp(id, "trailer") == 0 ? "trailer.0" : id;
    for (uint32_t i = 0; i < kConfigSectionCount; ++i) {
        if (std::strcmp(kConfigSections[i].id, key) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

SCSAPI_VOID on_configuration(const scs_event_t, const void* const info, const scs_context_t) {
    if (!g.cfg) {
        return;
    }
    const auto* conf = static_cast<const scs_telemetry_configuration_t*>(info);
    const int si = find_section(conf->id);
    if (si < 0) {
        return;  // substances, hshifter, bus_job, ... are not mirrored
    }
    const ConfigSectionDef& sec = kConfigSections[si];
    uint8_t* base = reinterpret_cast<uint8_t*>(g.cfg);
    uint8_t* region = base + sec.offset;

    volatile uint64_t* seq = &g.cfg->config_seq;
    *seq = *seq + 1;  // odd: writing
    std::atomic_thread_fence(std::memory_order_release);

    uint32_t section_seq;
    std::memcpy(&section_seq, region, 4);  // every section struct starts with u32 seq, u8 present
    std::memset(region, 0, sec.size);
    std::string key;
    for (const scs_named_value_t* a = conf->attributes; a && a->name; ++a) {
        key.assign(std::to_string(si)).append("|").append(a->name);
        auto it = g.attr_lookup.find(key);
        if (it == g.attr_lookup.end()) {
            continue;
        }
        for (uint32_t ai : it->second) {
            const ConfigAttrDef& def = kConfigAttrs[ai];
            if (def.index != a->index || def.scs_type != a->value.type) {
                continue;
            }
            store_value(base, def.offset, def.type, def.size, a->value);
        }
    }
    section_seq += 1;
    std::memcpy(region, &section_seq, 4);
    region[4] = (conf->attributes && conf->attributes->name) ? 1 : 0;  // present

    std::atomic_thread_fence(std::memory_order_release);
    *seq = *seq + 1;  // even: stable
}

void append_attr(std::string& out, const scs_named_value_t& a) {
    char buf[160];
    const scs_value_t& v = a.value;
    switch (v.type) {
        case SCS_VALUE_TYPE_bool: std::snprintf(buf, sizeof buf, "%s=%d", a.name, v.value_bool.value ? 1 : 0); break;
        case SCS_VALUE_TYPE_s32: std::snprintf(buf, sizeof buf, "%s=%d", a.name, v.value_s32.value); break;
        case SCS_VALUE_TYPE_u32: std::snprintf(buf, sizeof buf, "%s=%u", a.name, v.value_u32.value); break;
        case SCS_VALUE_TYPE_u64: std::snprintf(buf, sizeof buf, "%s=%llu", a.name, static_cast<unsigned long long>(v.value_u64.value)); break;
        case SCS_VALUE_TYPE_s64: std::snprintf(buf, sizeof buf, "%s=%lld", a.name, static_cast<long long>(v.value_s64.value)); break;
        case SCS_VALUE_TYPE_float: std::snprintf(buf, sizeof buf, "%s=%.6g", a.name, v.value_float.value); break;
        case SCS_VALUE_TYPE_double: std::snprintf(buf, sizeof buf, "%s=%.9g", a.name, v.value_double.value); break;
        case SCS_VALUE_TYPE_string: std::snprintf(buf, sizeof buf, "%s=%s", a.name, v.value_string.value ? v.value_string.value : ""); break;
        default: std::snprintf(buf, sizeof buf, "%s=?", a.name); break;
    }
    // ';' and '=' are separators; strip them from values.
    for (char* p = std::strchr(buf, '=') + 1; *p; ++p) {
        if (*p == ';') {
            *p = ',';
        }
    }
    if (!out.empty()) {
        out += ';';
    }
    out += buf;
}

SCSAPI_VOID on_gameplay(const scs_event_t, const void* const info, const scs_context_t) {
    if (!g.hdr) {
        return;
    }
    const auto* ev = static_cast<const scs_telemetry_gameplay_event_t*>(info);
    std::string attrs;
    uint32_t count = 0;
    for (const scs_named_value_t* a = ev->attributes; a && a->name; ++a, ++count) {
        append_attr(attrs, *a);
    }
    const uint64_t n = ++g.event_seq;
    EventSlot* slot = &g.events[(n - 1) % EVENT_SLOTS];
    volatile uint64_t* seq = &slot->seq;
    *seq = 0;
    std::atomic_thread_fence(std::memory_order_release);
    slot->game_time = g.work.game_time;
    slot->attr_count = count;
    copy_str(slot->id, sizeof slot->id, ev->id);
    copy_str(slot->attrs, sizeof slot->attrs, attrs.c_str());
    std::atomic_thread_fence(std::memory_order_release);
    *seq = n;
    g.hdr->event_write_index = n;
}

// ------------------------------------------------------------------ input

bool key_down(int vk) { return (GetAsyncKeyState(vk) & 0x8000) != 0; }

bool kill_key_down(const ControlsBlock* snap) {
    if (g_test_kill.load(std::memory_order_relaxed)) {
        return true;
    }
    for (int vk : kDefaultKillVk) {
        if (key_down(vk)) {
            return true;
        }
    }
    if (snap) {
        for (uint32_t i = 0; i < MAX_EXTRA_KILL_VK; ++i) {
            if (snap->header.extra_kill_vk[i] && key_down(snap->header.extra_kill_vk[i])) {
                return true;
            }
        }
    }
    return false;
}

void queue_axis(uint32_t i, float v) {
    scs_input_event_t& e = g.queue[g.queue_len++];
    std::memset(&e, 0, sizeof e);
    e.input_index = i;
    e.value_float.value = v;
}

void queue_button(uint32_t j, bool v) {
    scs_input_event_t& e = g.queue[g.queue_len++];
    std::memset(&e, 0, sizeof e);
    e.input_index = NUM_AXES + j;
    e.value_bool.value = v ? 1 : 0;
}

void mark_delivered(const scs_input_event_t& e);

// After (re)activation the game may have lost or reset the device's values. Re-send every axis and any
// button that is or was pressed; released buttons need nothing (8 events instead of all 85).
void force_resync(const InputState* target) {
    for (uint32_t i = 0; i < NUM_AXES; ++i) {
        g.force[i] = true;
    }
    for (uint32_t j = 0; j < NUM_BUTTONS; ++j) {
        if (g.sent.buttons[j] || (target && target->buttons[j])) {
            g.force[NUM_AXES + j] = true;
        }
    }
}

void prepare_input_frame(bool resend_all) {
    if (g.queue_pos < g.queue_len) {
        g.hdr->input_events_requeued += g.queue_len - g.queue_pos;   // offered last frame, never fetched
    }
    g.queue_len = 0;
    g.queue_pos = 0;
    const uint64_t now = qpc_now();

    // Snapshot the client's block (single writer: the Python process).
    ControlsBlock snap;
    std::memcpy(&snap, g.controls, sizeof snap);
    const bool client_ok = snap.header.magic == kControlsMagic && snap.header.layout_hash == kLayoutHash;

    if (client_ok && (!g.client_known || snap.header.client_pid != g.client_pid)) {
        // New client. Pulse counters live in the block and a client continues from the values it
        // found there, so seen_pulses (adopted once when the DLL opened the block) stays valid:
        // presses the client sends in its very first commit still fire.
        g.last_rearm = snap.header.rearm_counter;
        g.client_known = true;
        g.client_pid = snap.header.client_pid;
        log_runtime("controls client connected (%s)", "layout ok");
    } else if (!client_ok) {
        g.client_known = false;
    }

    const bool kill_down = kill_key_down(client_ok ? &snap : nullptr);
    if (kill_down && !g.kill_latched) {
        g.kill_latched = true;
        g.hdr->kill_count += 1;
        log_runtime("kill switch: %s", "all inputs released");
    }
    if (client_ok && snap.header.rearm_counter != g.last_rearm) {
        g.last_rearm = snap.header.rearm_counter;
        if (!kill_down && g.kill_latched) {
            g.kill_latched = false;
            log_runtime("kill switch: %s", "re-armed by client");
        }
    }

    uint32_t wd_ms = client_ok && snap.header.watchdog_ms ? snap.header.watchdog_ms : kDefaultWatchdogMs;
    wd_ms = wd_ms < kMinWatchdogMs ? kMinWatchdogMs : (wd_ms > kMaxWatchdogMs ? kMaxWatchdogMs : wd_ms);
    g.hdr->watchdog_ms = wd_ms;
    const uint64_t wd_ticks = g.qpc_freq * wd_ms / 1000;
    const bool fresh = snap.header.heartbeat_qpc != 0 && now >= snap.header.heartbeat_qpc
                       && (now - snap.header.heartbeat_qpc) <= wd_ticks;
    const bool alive = client_ok && snap.header.enable == 1 && fresh;
    if (g.was_alive && !alive) {
        if (client_ok && snap.header.enable == 1) {
            g.hdr->watchdog_trips += 1;
            log_runtime("watchdog: %s", "heartbeat lost, all inputs released");
        }
    }
    g.was_alive = alive;
    const bool active = alive && !g.kill_latched && !g.paused;

    if (client_ok) {
        g.hdr->applied_write_seq = snap.header.write_seq;
    }
    g.hdr->input_frames += 1;
    g.hdr->last_input_qpc = now;
    set_flag(BRIDGE_FLAG_KILL_LATCHED, g.kill_latched);
    set_flag(BRIDGE_FLAG_CONTROL_ALIVE, alive);
    set_flag(BRIDGE_FLAG_CONTROL_ACTIVE, active);

    InputState target;
    for (uint32_t i = 0; i < NUM_AXES; ++i) {
        float v = active ? snap.axes[i] : 0.0f;
        if (!std::isfinite(v)) {
            v = 0.0f;
        }
        target.axes[i] = v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
    }
    for (uint32_t j = 0; j < NUM_BUTTONS; ++j) {
        if (client_ok && snap.pulses[j] != g.seen_pulses[j]) {
            const uint32_t delta = snap.pulses[j] - g.seen_pulses[j];  // wraps correctly
            g.seen_pulses[j] = snap.pulses[j];
            if (active) {
                const uint32_t total = g.pending_pulses[j] + delta;
                g.pending_pulses[j] = total > kMaxPendingPulses ? kMaxPendingPulses : total;
            }
        }
        if (!active) {
            g.pending_pulses[j] = 0;
            g.pulse_phase[j] = 0;
        }
        // A pulse is one pressed frame then one released frame. Each step advances only once the game
        // has fetched it, so pulses survive a game that takes fewer events per frame than we offer.
        bool pulse_out = false;
        if (g.pulse_phase[j] == 1 && !g.sent.buttons[j]) {
            pulse_out = true;                // press not delivered yet: keep offering it
        } else if (g.pulse_phase[j] == 1) {
            g.pulse_phase[j] = 2;            // release after a delivered press
        } else if (g.pulse_phase[j] == 2 && g.sent.buttons[j] && !(active && snap.holds[j] != 0)) {
            // release not delivered yet: stay in phase 2 (target false)
        } else if (g.pending_pulses[j] > 0) {
            g.pending_pulses[j] -= 1;
            g.pulse_phase[j] = 1;
            pulse_out = true;
        } else {
            g.pulse_phase[j] = 0;
        }
        target.buttons[j] = (active && snap.holds[j] != 0) || pulse_out;
    }

    // Offer every input whose value differs from what the game last *fetched*. The 1.53 game was seen to
    // take fewer events per frame than offered; events it doesn't fetch stay dirty and are offered again
    // next frame, starting after the last delivered input so steering, throttle and brake take turns.
    if (resend_all) {
        force_resync(&target);
    }
    // Pass 1: inputs whose value changed (commands, releases after a kill) - always first, taking turns.
    // Pass 2: forced resyncs of unchanged values. A game that fetches one event per frame then never makes
    // a real change wait behind a resync.
    for (int pass = 0; pass < 2; ++pass) {
        for (uint32_t k = 0; k < NUM_INPUTS; ++k) {
            const uint32_t i = (g.rr_start + k) % NUM_INPUTS;
            bool changed;
            if (i < NUM_AXES) {
                changed = std::fabs(target.axes[i] - g.sent.axes[i]) > 1e-6f;
            } else {
                changed = target.buttons[i - NUM_AXES] != g.sent.buttons[i - NUM_AXES];
            }
            if (pass == 0 ? !changed : (changed || !g.force[i])) {
                continue;
            }
            if (i < NUM_AXES) {
                queue_axis(i, target.axes[i]);
            } else {
                queue_button(i - NUM_AXES, target.buttons[i - NUM_AXES]);
            }
        }
    }
}

void mark_delivered(const scs_input_event_t& e) {
    const uint32_t i = e.input_index;
    if (i < NUM_AXES) {
        g.sent.axes[i] = e.value_float.value;
    } else if (i < NUM_INPUTS) {
        g.sent.buttons[i - NUM_AXES] = e.value_bool.value != 0;
    }
    if (i < NUM_INPUTS) {
        g.force[i] = false;
        g.rr_start = (i + 1) % NUM_INPUTS;
    }
    g.hdr->input_events_delivered += 1;
}

SCSAPI_RESULT on_input_event(scs_input_event_t* const event_info, const scs_u32_t flags, const scs_context_t) {
    if (!g.hdr) {
        return SCS_RESULT_not_found;
    }
    if (flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame) {
        prepare_input_frame((flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_after_activation) != 0);
    }
    g.hdr->input_calls += 1;
    if (g.queue_pos < g.queue_len) {
        *event_info = g.queue[g.queue_pos++];
        mark_delivered(*event_info);
        return SCS_RESULT_ok;
    }
    return SCS_RESULT_not_found;
}

SCSAPI_VOID on_input_active(const scs_u8_t active, const scs_context_t) {
    if (g.hdr) {
        g.hdr->input_active = active ? 1 : 0;
    }
    if (!active) {
        force_resync(nullptr);  // re-send axes and pressed buttons on reactivation
    }
}

}  // namespace

// ------------------------------------------------------------------ exports

extern "C" {

SCSAPI_RESULT scs_telemetry_init(const scs_u32_t version, const scs_telemetry_init_params_t* const params) {
    if (version != SCS_TELEMETRY_VERSION_1_01 && version != SCS_TELEMETRY_VERSION_1_00) {
        return SCS_RESULT_unsupported;
    }
    const auto* p = static_cast<const scs_telemetry_init_params_v100_t*>(params);
    g.log = p->common.log;
    if (!ensure_shared_memory()) {
        return SCS_RESULT_generic_error;
    }

    bool ok = p->register_for_event(SCS_TELEMETRY_EVENT_frame_start, on_frame_start, nullptr) == SCS_RESULT_ok;
    ok = ok && p->register_for_event(SCS_TELEMETRY_EVENT_frame_end, on_frame_end, nullptr) == SCS_RESULT_ok;
    ok = ok && p->register_for_event(SCS_TELEMETRY_EVENT_paused, on_pause_change, nullptr) == SCS_RESULT_ok;
    ok = ok && p->register_for_event(SCS_TELEMETRY_EVENT_started, on_pause_change, nullptr) == SCS_RESULT_ok;
    ok = ok && p->register_for_event(SCS_TELEMETRY_EVENT_configuration, on_configuration, nullptr) == SCS_RESULT_ok;
    if (!ok) {
        logf(SCS_LOG_TYPE_error, "cannot register telemetry events");
        return SCS_RESULT_generic_error;
    }
    if (p->register_for_event(SCS_TELEMETRY_EVENT_gameplay, on_gameplay, nullptr) != SCS_RESULT_ok) {
        logf(SCS_LOG_TYPE_warning, "gameplay events unavailable (telemetry API %u.%u)",
             SCS_GET_MAJOR_VERSION(version), SCS_GET_MINOR_VERSION(version));
    }

    g.attr_lookup.clear();
    for (uint32_t i = 0; i < kConfigAttrCount; ++i) {
        g.attr_lookup[std::to_string(kConfigAttrs[i].section) + "|" + kConfigAttrs[i].name].push_back(i);
    }

    std::memset(&g.work, 0, sizeof g.work);
    g.work.next_mandatory_break = INT32_MIN;  // stays at the sentinel when the channel doesn't exist (1.53)
    g.channels_ok = 0;
    std::string missing;
    for (uint32_t id = 0; id < kChannelCount; ++id) {
        const ChannelDef& c = kChannels[id];
        const scs_result_t r = p->register_for_channel(c.name, c.index, c.scs_type, SCS_TELEMETRY_CHANNEL_FLAG_none,
                                                       on_channel, reinterpret_cast<scs_context_t>(static_cast<uintptr_t>(id)));
        if (r == SCS_RESULT_ok) {
            ++g.channels_ok;
        } else if (c.index == SCS_U32_NIL && missing.size() < 300) {
            missing += missing.empty() ? "" : ", ";
            missing += c.name;
        }
    }

    TelemetryHeader& h = *g.hdr;
    h.telemetry_api_version = version;
    h.telemetry_game_version = p->common.game_version;
    copy_str(h.game_name, sizeof h.game_name, p->common.game_name);
    copy_str(h.game_id, sizeof h.game_id, p->common.game_id);
    set_flag(BRIDGE_FLAG_TELEMETRY_OK, true);
    g.telemetry_on = true;
    g.paused = true;

    logf(SCS_LOG_TYPE_message, "bridge %s, layout v%u (0x%016llx): %s telemetry game version %u.%u, %u/%u channels",
         kBridgeVersion, kLayoutVersion, static_cast<unsigned long long>(kLayoutHash), p->common.game_id,
         SCS_GET_MAJOR_VERSION(p->common.game_version), SCS_GET_MINOR_VERSION(p->common.game_version),
         g.channels_ok, kChannelCount);
    if (!missing.empty()) {
        logf(SCS_LOG_TYPE_message, "channels not provided by this game version: %s", missing.c_str());
    }
    return SCS_RESULT_ok;
}

SCSAPI_VOID scs_telemetry_shutdown(void) {
    g.telemetry_on = false;
    set_flag(BRIDGE_FLAG_TELEMETRY_OK, false);
    g.log = nullptr;
}

SCSAPI_RESULT scs_input_init(const scs_u32_t version, const scs_input_init_params_t* const params) {
    if (version != SCS_INPUT_VERSION_1_00) {
        return SCS_RESULT_unsupported;
    }
    const auto* p = static_cast<const scs_input_init_params_v100_t*>(params);
    if (!g.log) {
        g.log = p->common.log;
    }
    if (!ensure_shared_memory()) {
        return SCS_RESULT_generic_error;
    }
    for (uint32_t i = 0; i < NUM_INPUTS; ++i) {
        g.input_defs[i].name = kInputs[i].name;
        g.input_defs[i].display_name = kInputs[i].display;
        g.input_defs[i].value_type = kInputs[i].is_axis ? SCS_VALUE_TYPE_float : SCS_VALUE_TYPE_bool;
    }
    scs_input_device_t dev;
    std::memset(&dev, 0, sizeof dev);
    dev.name = "autohaul";
    dev.display_name = "AutoHaul";
    dev.type = SCS_INPUT_DEVICE_TYPE_semantical;
    dev.input_count = NUM_INPUTS;
    dev.inputs = g.input_defs;
    dev.callback_context = nullptr;
    dev.input_active_callback = on_input_active;
    dev.input_event_callback = on_input_event;
    if (p->register_device(&dev) != SCS_RESULT_ok) {
        logf(SCS_LOG_TYPE_error, "cannot register semantical input device");
        return SCS_RESULT_generic_error;
    }
    g.sent = InputState{};
    force_resync(nullptr);
    g.client_known = false;
    g.was_alive = false;
    g.hdr->input_api_version = version;
    g.hdr->input_game_version = p->common.game_version;
    g.hdr->input_registered = 1;
    set_flag(BRIDGE_FLAG_INPUT_OK, true);
    g.input_on = true;
    logf(SCS_LOG_TYPE_message, "semantical input device registered: %u axes, %u buttons, watchdog %u ms",
         NUM_AXES, NUM_BUTTONS, kDefaultWatchdogMs);
    return SCS_RESULT_ok;
}

SCSAPI_VOID scs_input_shutdown(void) {
    g.input_on = false;
    if (g.hdr) {
        g.hdr->input_registered = 0;
        g.hdr->input_active = 0;
    }
    set_flag(BRIDGE_FLAG_INPUT_OK, false);
}

// Test hook used only by tests/fake_game.cpp: simulates holding a kill key.
__declspec(dllexport) void __cdecl autohaul_test_force_kill_key(int down) { g_test_kill.store(down ? 1 : 0); }

}  // extern "C"

BOOL APIENTRY DllMain(HMODULE, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_DETACH) {
        if (g.frame_event) {
            CloseHandle(g.frame_event);
            g.frame_event = nullptr;
        }
        close_mapping(g.tel);
        close_mapping(g.ctl);
        g.hdr = nullptr;
    }
    return TRUE;
}
