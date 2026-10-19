"""Windows virtual-key names used in config (kill keys, override keys)."""

from __future__ import annotations

VK: dict[str, int] = {
    "BACKSPACE": 0x08, "TAB": 0x09, "RETURN": 0x0D, "ENTER": 0x0D, "SHIFT": 0x10, "CONTROL": 0x11, "CTRL": 0x11,
    "ALT": 0x12, "PAUSE": 0x13, "CAPSLOCK": 0x14, "ESCAPE": 0x1B, "ESC": 0x1B, "SPACE": 0x20,
    "PAGEUP": 0x21, "PAGEDOWN": 0x22, "END": 0x23, "HOME": 0x24,
    "LEFT": 0x25, "UP": 0x26, "RIGHT": 0x27, "DOWN": 0x28,
    "INSERT": 0x2D, "DELETE": 0x2E, "SCROLL": 0x91, "SCROLLLOCK": 0x91, "NUMLOCK": 0x90,
    "LSHIFT": 0xA0, "RSHIFT": 0xA1, "LCONTROL": 0xA2, "RCONTROL": 0xA3, "LALT": 0xA4, "RALT": 0xA5,
    "SEMICOLON": 0xBA, "EQUALS": 0xBB, "COMMA": 0xBC, "MINUS": 0xBD, "PERIOD": 0xBE, "SLASH": 0xBF,
    "GRAVE": 0xC0, "LBRACKET": 0xDB, "BACKSLASH": 0xDC, "RBRACKET": 0xDD, "APOSTROPHE": 0xDE,
    "OEM_102": 0xE2,
}
VK.update({chr(c): c for c in range(ord("A"), ord("Z") + 1)})
VK.update({str(d): 0x30 + d for d in range(10)})
VK.update({f"F{n}": 0x6F + n for n in range(1, 25)})
VK.update({f"NUMPAD{d}": 0x60 + d for d in range(10)})

VK_NAME = {v: k for k, v in reversed(list(VK.items()))}


def vk(name: str) -> int:
    try:
        return VK[name.upper()]
    except KeyError:
        raise KeyError(f"unknown virtual-key name {name!r}") from None


def vks(names: list[str]) -> tuple[int, ...]:
    return tuple(vk(n) for n in names)
