// fake_game.exe: loads autohaul_bridge.dll and plays the game's side of the SCS SDK so the
// bridge can be tested end to end without ETS2.
//
//   fake_game.exe <dll> [--frames N] [--hz H] [--game-version MAJOR.MINOR] [--drop CHANNEL]...
//                       [--kill-at FRAME] [--kill-frames N] [--pause-at FRAME] [--resume-at FRAME]
//                       [--parked] [--convoy] [--telemetry-only]
//
// stdout protocol (one line each, flushed every frame):
//   READY                      after both SDK inits succeeded
//   LOG <type> <message>       SDK log calls from the DLL
//   INPUT <frame> <index> <v>  every input event the device produced
//   FRAME <frame>              end of a frame
//   DONE

#include <windows.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "scssdk_input.h"
#include "scssdk_input_device.h"
#include "scssdk_telemetry.h"
#include "eurotrucks2/scssdk_eut2.h"

namespace {

struct ChannelReg {
    std::string name;
    scs_u32_t index;
    scs_value_type_t type;
    scs_telemetry_channel_callback_t cb;
    scs_context_t ctx;
};

struct EventReg {
    scs_telemetry_event_callback_t cb = nullptr;
    scs_context_t ctx = nullptr;
};

std::vector<ChannelReg> g_channels;
EventReg g_events[8];
std::set<std::string> g_drop;
scs_input_device_t g_device{};
std::vector<scs_input_device_input_t> g_inputs;
std::vector<std::string> g_input_names;
bool g_have_device = false;

// --parked: a stationary single-player truck that echoes the device's inputs into telemetry
// (1 frame later, like the real game), so tools/phase1_check.py can be dry-run.
struct SimTruck {
    bool parked = false;
    bool convoy = false;
    float steer = 0.0f, throttle = 0.0f, brake = 0.0f;
    bool lblinker = false, rblinker = false, hazard = false, wipers = false;
    int light = 0;  // 0 off, 1 parking, 2 low beam
} g_sim;

void sim_apply(const scs_input_event_t& ev) {
    const std::string& n = g_input_names[ev.input_index];
    const bool b = ev.value_bool.value != 0;
    if (n == "steering") g_sim.steer = ev.value_float.value;
    else if (n == "aforward") g_sim.throttle = ev.value_float.value;
    else if (n == "abackward") g_sim.brake = ev.value_float.value;
    else if (!b) return;  // buttons act on press
    else if (n == "lblinker") g_sim.lblinker = !g_sim.lblinker;
    else if (n == "rblinker") g_sim.rblinker = !g_sim.rblinker;
    else if (n == "flasher4way") g_sim.hazard = !g_sim.hazard;
    else if (n == "light") g_sim.light = (g_sim.light + 1) % 3;
    else if (n == "wipers") g_sim.wipers = !g_sim.wipers;
    else if (n == "wipers0") g_sim.wipers = false;
    else if (n == "wipers1" || n == "wipers2" || n == "wipers3" || n == "wipers4") g_sim.wipers = true;
}

// Returns true and fills `v` when the simulated truck overrides this channel.
bool sim_value(const ChannelReg& c, scs_value_t& v) {
    if (c.name == "multiplayer.time.offset") {
        v.value_s32.value = g_sim.convoy ? 37 : 0;
        return true;
    }
    if (!g_sim.parked) return false;
    const std::string& n = c.name;
    auto f = [&](float x) { v.value_float.value = x; return true; };
    auto bl = [&](bool x) { v.value_bool.value = x ? 1 : 0; return true; };
    if (n == "truck.speed") return f(0.0f);
    if (n == "truck.brake.parking") return bl(true);
    if (n == "truck.input.steering") return f(-g_sim.steer);
    if (n == "truck.effective.steering") return f(-0.9f * g_sim.steer);
    if (n == "truck.input.throttle") return f(g_sim.throttle);
    if (n == "truck.input.brake") return f(g_sim.brake);
    if (n == "truck.wheel.steering") return f(c.index < 2 ? -0.1f * g_sim.steer : 0.0f);
    if (n == "truck.lblinker") return bl(g_sim.lblinker);
    if (n == "truck.rblinker") return bl(g_sim.rblinker);
    if (n == "truck.hazard.warning") return bl(g_sim.hazard);
    if (n == "truck.light.parking") return bl(g_sim.light >= 1);
    if (n == "truck.light.beam.low") return bl(g_sim.light == 2);
    if (n == "truck.wipers") return bl(g_sim.wipers);
    if (c.type == SCS_VALUE_TYPE_bool && n != "trailer.0.connected") return bl(false);
    return false;
}

SCSAPI_VOID fake_log(const scs_log_type_t type, const scs_string_t message) {
    std::printf("LOG %d %s\n", static_cast<int>(type), message);
    std::fflush(stdout);
}

SCSAPI_RESULT fake_register_event(const scs_event_t event, const scs_telemetry_event_callback_t cb, const scs_context_t ctx) {
    if (event >= 8) {
        return SCS_RESULT_unsupported;
    }
    g_events[event] = {cb, ctx};
    return SCS_RESULT_ok;
}

SCSAPI_RESULT fake_unregister_event(const scs_event_t event) {
    if (event < 8) {
        g_events[event] = {};
    }
    return SCS_RESULT_ok;
}

SCSAPI_RESULT fake_register_channel(const scs_string_t name, const scs_u32_t index, const scs_value_type_t type,
                                    const scs_u32_t, const scs_telemetry_channel_callback_t cb, const scs_context_t ctx) {
    if (g_drop.count(name)) {
        return SCS_RESULT_not_found;  // simulates a channel this game version doesn't have
    }
    g_channels.push_back({name, index, type, cb, ctx});
    return SCS_RESULT_ok;
}

SCSAPI_RESULT fake_unregister_channel(const scs_string_t, const scs_u32_t, const scs_value_type_t) { return SCS_RESULT_ok; }

SCSAPI_RESULT fake_register_device(const scs_input_device_t* const dev) {
    g_device = *dev;
    g_inputs.assign(dev->inputs, dev->inputs + dev->input_count);
    for (const auto& in : g_inputs) {
        g_input_names.emplace_back(in.name);  // copy: the DLL's strings must stay valid anyway, but be safe
    }
    g_have_device = true;
    return SCS_RESULT_ok;
}

void fire(scs_event_t e, const void* info) {
    if (g_events[e].cb) {
        g_events[e].cb(e, info, g_events[e].ctx);
    }
}

scs_named_value_t nv_string(const char* name, const char* v, scs_u32_t index = SCS_U32_NIL) {
    scs_named_value_t n{};
    n.name = name;
    n.index = index;
    n.value.type = SCS_VALUE_TYPE_string;
    n.value.value_string.value = v;
    return n;
}
scs_named_value_t nv_u32(const char* name, scs_u32_t v, scs_u32_t index = SCS_U32_NIL) {
    scs_named_value_t n{};
    n.name = name;
    n.index = index;
    n.value.type = SCS_VALUE_TYPE_u32;
    n.value.value_u32.value = v;
    return n;
}
scs_named_value_t nv_u64(const char* name, scs_u64_t v) {
    scs_named_value_t n{};
    n.name = name;
    n.index = SCS_U32_NIL;
    n.value.type = SCS_VALUE_TYPE_u64;
    n.value.value_u64.value = v;
    return n;
}
scs_named_value_t nv_s64(const char* name, scs_s64_t v) {
    scs_named_value_t n{};
    n.name = name;
    n.index = SCS_U32_NIL;
    n.value.type = SCS_VALUE_TYPE_s64;
    n.value.value_s64.value = v;
    return n;
}
scs_named_value_t nv_s32(const char* name, scs_s32_t v) {
    scs_named_value_t n{};
    n.name = name;
    n.index = SCS_U32_NIL;
    n.value.type = SCS_VALUE_TYPE_s32;
    n.value.value_s32.value = v;
    return n;
}
scs_named_value_t nv_float(const char* name, float v, scs_u32_t index = SCS_U32_NIL) {
    scs_named_value_t n{};
    n.name = name;
    n.index = index;
    n.value.type = SCS_VALUE_TYPE_float;
    n.value.value_float.value = v;
    return n;
}
scs_named_value_t nv_bool(const char* name, bool v, scs_u32_t index = SCS_U32_NIL) {
    scs_named_value_t n{};
    n.name = name;
    n.index = index;
    n.value.type = SCS_VALUE_TYPE_bool;
    n.value.value_bool.value = v ? 1 : 0;
    return n;
}
scs_named_value_t nv_fvector(const char* name, float x, float y, float z, scs_u32_t index = SCS_U32_NIL) {
    scs_named_value_t n{};
    n.name = name;
    n.index = index;
    n.value.type = SCS_VALUE_TYPE_fvector;
    n.value.value_fvector = {x, y, z};
    return n;
}
scs_named_value_t nv_end() { return scs_named_value_t{}; }

void send_config(const char* id, std::vector<scs_named_value_t> attrs) {
    attrs.push_back(nv_end());
    scs_telemetry_configuration_t conf{id, attrs.data()};
    fire(SCS_TELEMETRY_EVENT_configuration, &conf);
}

void send_gameplay(const char* id, std::vector<scs_named_value_t> attrs) {
    attrs.push_back(nv_end());
    scs_telemetry_gameplay_event_t ev{id, attrs.data()};
    fire(SCS_TELEMETRY_EVENT_gameplay, &ev);
}

void send_initial_config() {
    std::vector<scs_named_value_t> truck = {
        nv_string("brand_id", "scania"), nv_string("brand", "Scania"), nv_string("id", "vehicle.scania.s_2016"),
        nv_string("name", "S"), nv_float("fuel.capacity", 800.0f), nv_float("rpm.limit", 2500.0f),
        nv_u32("gears.forward", 12), nv_u32("gears.reverse", 2), nv_u32("retarder.steps", 5),
        nv_fvector("hook.position", 0.0f, 1.1f, 0.6f), nv_u32("wheels.count", 6),
    };
    for (scs_u32_t i = 0; i < 6; ++i) {
        truck.push_back(nv_fvector("wheel.position", (i % 2) ? 1.0f : -1.0f, 0.5f, -1.5f * static_cast<float>(i / 2), i));
        truck.push_back(nv_bool("wheel.steerable", i < 2, i));
        truck.push_back(nv_float("wheel.radius", 0.5f, i));
    }
    for (scs_u32_t i = 0; i < 12; ++i) {
        truck.push_back(nv_float("forward.ratio", 10.0f - 0.7f * static_cast<float>(i), i));
    }
    send_config("truck", truck);
    send_config("trailer.0", {nv_string("id", "scs_box"), nv_string("body.type", "dryvan"),
                              nv_fvector("hook.position", 0.0f, 1.2f, -5.0f), nv_u32("wheels.count", 3)});
    send_config("job", {nv_string("cargo.id", "apples"), nv_string("cargo", "Apples"), nv_float("cargo.mass", 20000.0f),
                        nv_string("destination.city", "Berlin"), nv_string("destination.company", "TREE-ET"),
                        nv_string("source.city", "Dresden"), nv_u64("income", 12345), nv_u32("delivery.time", 5000),
                        nv_u32("planned_distance.km", 190), nv_bool("cargo.loaded", true), nv_string("job.market", "cargo_market")});
    send_config("controls", {nv_string("shifter.type", "automatic")});
    send_config("substances", {nv_string("id", "asphalt", 0)});
}

void send_channels(int frame) {
    for (const auto& c : g_channels) {
        scs_value_t v{};
        v.type = c.type;
        const float f = static_cast<float>(frame);
        switch (c.type) {
            case SCS_VALUE_TYPE_bool: v.value_bool.value = (c.name == "trailer.0.connected") ? 1 : 0; break;
            case SCS_VALUE_TYPE_s32: v.value_s32.value = (c.name == "rest.stop") ? 600 - frame / 10 : -3; break;
            case SCS_VALUE_TYPE_u32: v.value_u32.value = (c.name == "game.time") ? 1000u + static_cast<scs_u32_t>(frame / 10) : 7u; break;
            case SCS_VALUE_TYPE_u64: v.value_u64.value = 9; break;
            case SCS_VALUE_TYPE_float:
                v.value_float.value = (c.name == "truck.speed") ? 10.0f + 0.01f * f
                                      : (c.index != SCS_U32_NIL) ? 0.1f * static_cast<float>(c.index)
                                                                 : 0.5f;
                break;
            case SCS_VALUE_TYPE_double: v.value_double.value = 0.25; break;
            case SCS_VALUE_TYPE_fvector: v.value_fvector = {1.0f, 2.0f, 3.0f}; break;
            case SCS_VALUE_TYPE_fplacement: v.value_fplacement = {{1.0f, 2.0f, 3.0f}, {0.1f, 0.2f, 0.3f}}; break;
            case SCS_VALUE_TYPE_dplacement:
                v.value_dplacement.position = {100.0 + frame, 5.0, -200.0 - frame};
                v.value_dplacement.orientation = {0.25f, 0.01f, 0.0f};
                break;
            default: continue;
        }
        sim_value(c, v);
        c.cb(c.name.c_str(), c.index, &v, c.ctx);
    }
}

void poll_input(int frame, bool after_activation) {
    if (!g_have_device) {
        return;
    }
    scs_u32_t flags = SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame;
    if (after_activation) {
        flags |= SCS_INPUT_EVENT_CALLBACK_FLAG_first_after_activation;
    }
    for (int guard = 0; guard < 1000; ++guard) {
        scs_input_event_t ev{};
        const scs_result_t r = g_device.input_event_callback(&ev, flags, g_device.callback_context);
        if (r == SCS_RESULT_not_found) {
            break;
        }
        if (r != SCS_RESULT_ok) {
            std::printf("ERROR input callback returned %d\n", r);
            break;
        }
        flags = 0;
        sim_apply(ev);
        const bool is_float = g_inputs[ev.input_index].value_type == SCS_VALUE_TYPE_float;
        if (is_float) {
            std::printf("INPUT %d %u %.6f\n", frame, ev.input_index, ev.value_float.value);
        } else {
            std::printf("INPUT %d %u %d\n", frame, ev.input_index, ev.value_bool.value ? 1 : 0);
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: fake_game <dll> [options]\n");
        return 2;
    }
    int frames = 300, hz = 60, kill_at = -1, kill_frames = 10, pause_at = -1, resume_at = -1;
    bool telemetry_only = false;  // load any telemetry-only plugin (e.g. ets2_la_plugin.dll)
    unsigned major = 1, minor = 18;
    g_drop.insert("mandatory.break");  // like 1.53 (telemetry game version 1.18)
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--frames") frames = std::atoi(next());
        else if (a == "--hz") hz = std::atoi(next());
        else if (a == "--game-version") sscanf_s(next(), "%u.%u", &major, &minor);
        else if (a == "--drop") g_drop.insert(next());
        else if (a == "--keep-mandatory-break") g_drop.erase("mandatory.break");
        else if (a == "--kill-at") kill_at = std::atoi(next());
        else if (a == "--kill-frames") kill_frames = std::atoi(next());
        else if (a == "--pause-at") pause_at = std::atoi(next());
        else if (a == "--resume-at") resume_at = std::atoi(next());
        else if (a == "--parked") g_sim.parked = true;
        else if (a == "--telemetry-only") telemetry_only = true;
        else if (a == "--convoy") g_sim.convoy = true;
    }

    HMODULE dll = LoadLibraryA(argv[1]);
    if (!dll) {
        std::fprintf(stderr, "cannot load %s (error %lu)\n", argv[1], GetLastError());
        return 1;
    }
    using tinit_t = scs_result_t(SCSAPIFUNC*)(scs_u32_t, const scs_telemetry_init_params_t*);
    using iinit_t = scs_result_t(SCSAPIFUNC*)(scs_u32_t, const scs_input_init_params_t*);
    using shut_t = void(SCSAPIFUNC*)();
    using kill_t = void(__cdecl*)(int);
    auto tinit = reinterpret_cast<tinit_t>(GetProcAddress(dll, "scs_telemetry_init"));
    auto tshut = reinterpret_cast<shut_t>(GetProcAddress(dll, "scs_telemetry_shutdown"));
    auto iinit = reinterpret_cast<iinit_t>(GetProcAddress(dll, "scs_input_init"));
    auto ishut = reinterpret_cast<shut_t>(GetProcAddress(dll, "scs_input_shutdown"));
    auto force_kill = reinterpret_cast<kill_t>(GetProcAddress(dll, "autohaul_test_force_kill_key"));
    if (!tinit || !tshut || (!telemetry_only && (!iinit || !ishut || !force_kill))) {
        std::fprintf(stderr, "missing exports\n");
        return 1;
    }

    scs_telemetry_init_params_v101_t tp{};
    tp.common.game_name = "Euro Truck Simulator 2 (fake)";
    tp.common.game_id = SCS_GAME_ID_EUT2;
    tp.common.game_version = SCS_MAKE_VERSION(major, minor);
    tp.common.log = fake_log;
    tp.register_for_event = fake_register_event;
    tp.unregister_from_event = fake_unregister_event;
    tp.register_for_channel = fake_register_channel;
    tp.unregister_from_channel = fake_unregister_channel;
    if (tinit(SCS_TELEMETRY_VERSION_1_01, &tp) != SCS_RESULT_ok) {
        std::fprintf(stderr, "telemetry init failed\n");
        return 1;
    }

    scs_input_init_params_v100_t ip{};
    ip.common = tp.common;
    ip.common.game_version = SCS_MAKE_VERSION(1, 0);
    ip.register_device = fake_register_device;
    if (!telemetry_only && iinit(SCS_INPUT_VERSION_1_00, &ip) != SCS_RESULT_ok) {
        std::fprintf(stderr, "input init failed\n");
        return 1;
    }

    send_initial_config();
    fire(SCS_TELEMETRY_EVENT_started, nullptr);
    if (g_device.input_active_callback) {
        g_device.input_active_callback(1, g_device.callback_context);
    }
    std::printf("READY\n");
    std::fflush(stdout);

    LARGE_INTEGER freq, t0, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t0);
    bool after_activation = true;
    double fe_total_us = 0.0, fe_max_us = 0.0;
    for (int f = 1; f <= frames; ++f) {
        if (force_kill && f == kill_at) force_kill(1);
        if (force_kill && kill_at > 0 && f == kill_at + kill_frames) force_kill(0);
        if (f == pause_at) fire(SCS_TELEMETRY_EVENT_paused, nullptr);
        if (f == resume_at) fire(SCS_TELEMETRY_EVENT_started, nullptr);

        scs_telemetry_frame_start_t fs{};
        fs.render_time = static_cast<scs_timestamp_t>(f) * 1000000ull / static_cast<unsigned>(hz);
        fs.simulation_time = fs.render_time;
        fire(SCS_TELEMETRY_EVENT_frame_start, &fs);
        send_channels(f);
        if (f == 30 || f == 60) {
            send_gameplay("player.fined", {nv_string("fine.offence", "speeding"), nv_s64("fine.amount", 250)});
        }
        if (f == 90) {
            send_gameplay("job.delivered", {nv_s64("revenue", 12345), nv_s32("earned.xp", 500),
                                            nv_float("cargo.damage", 0.01f), nv_float("distance.km", 512.5f),
                                            nv_u32("delivery.time", 300), nv_bool("auto.park.used", true),
                                            nv_bool("auto.load.used", false)});
        }
        if (f == frames / 2) {
            send_config("job", {});  // job finished: empty attribute list
        }
        LARGE_INTEGER fe0, fe1;
        QueryPerformanceCounter(&fe0);
        fire(SCS_TELEMETRY_EVENT_frame_end, nullptr);
        QueryPerformanceCounter(&fe1);
        const double fe_us = static_cast<double>(fe1.QuadPart - fe0.QuadPart) * 1e6 / static_cast<double>(freq.QuadPart);
        fe_total_us += fe_us;
        if (fe_us > fe_max_us) fe_max_us = fe_us;

        poll_input(f, after_activation);
        after_activation = false;
        std::printf("FRAME %d\n", f);
        std::fflush(stdout);

        // Pace to the requested rate.
        const long long target = t0.QuadPart + freq.QuadPart * f / hz;
        for (;;) {
            QueryPerformanceCounter(&now);
            const long long left_ms = (target - now.QuadPart) * 1000 / freq.QuadPart;
            if (left_ms <= 0) break;
            Sleep(left_ms > 2 ? static_cast<DWORD>(left_ms - 1) : 0);
        }
    }

    std::printf("TIMING frame_end avg_us %.1f max_us %.1f\n", fe_total_us / frames, fe_max_us);
    if (!telemetry_only) ishut();
    tshut();
    FreeLibrary(dll);
    std::printf("DONE\n");
    std::fflush(stdout);
    return 0;
}
