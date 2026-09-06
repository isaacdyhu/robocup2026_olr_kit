#!/usr/bin/env pybricks-micropython
"""
pybricks/hub_camera_test.py -- test the Hub 1 <-> camera PUPRemote link.

Left button -> zone mode off (LINE mode). Right button -> zone mode on
(ZONE mode). Sends whichever one is currently selected on every loop
tick, not just once on the button press -- a level, not an edge-triggered
event, matching design.md Sec3's reasoning for the whole hub1<->hub2
protocol (broadcast/polled links are lossy, so a repeated level survives
a dropped message where a one-shot command wouldn't; the same logic
applies here even though this is a polled call rather than BLE
broadcast).

Camera on port D (design.md's Hub 1 table).

SELF-CONTAINED, DELIBERATELY: this used to `from pupremote_hub import
PUPRemoteHub`, relying on Pybricks Code auto-bundling a second local
file. Checked directly against pybricks/support#189 on GitHub -- that
"magic" multi-file bundling is confirmed beta-only (Pybricks Code Beta
or the pybricksdev CLI), and stable code.pybricks.com's own release
history has no mention of it ever landing there. Since this project is
being run from stable code.pybricks.com, the PUPRemoteHub class itself
(from github.com/antonvh/PUPRemote's pupremote_hub.py, GPL-licensed) is
copied in below instead, so there is no cross-file import to fail.

lpf2.py is NOT needed here and was removed from pybricks/ -- checked
directly against pupremote_hub.py's own source: it has zero references
to lpf2 anywhere, since the hub side talks over the link via Pybricks'
own built-in PUPDevice class (pybricks.iodevices), not a separate LPF2
implementation. lpf2.py is only needed on the OpenMV/camera side
(openmv_lib/), which uses the fuller pupremote.py instead.

UNTESTED END TO END: this and camera.py's "mode" command have never
actually been run together. add_command()'s name and both format
strings below must match camera.py's "mode" registration exactly, or
the two sides won't agree on how to interpret the bytes on the wire.
"""

# --- vendored from github.com/antonvh/PUPRemote (pupremote_hub.py, GPL) ----
#
# Trimmed to just what this script uses: PUPRemote/PUPRemoteHub and
# call()/add_command(). The multitask/async methods (call_multitask(),
# process_async()) are left out entirely, not merely unused, since this
# script only ever uses the plain synchronous call() -- see call()'s own
# assertion that it must NOT be used from inside a multitask context.
# Type hints referencing `Any` (never imported in the original file
# either) are dropped rather than trusted to be silently ignored.

import ustruct as struct
from pybricks.iodevices import PUPDevice
from pybricks.tools import wait, run_task
from micropython import const

MAX_PKT = const(16)

NAME = const(0)
SIZE = const(1)
TO_HUB_FORMAT = const(2)
FROM_HUB_FORMAT = const(3)
ARGS_TO_HUB = const(5)
ARGS_FROM_HUB = const(6)
CALLBACK = const(0)
CHANNEL = const(1)


class PUPRemote:
    """Base class for PUPRemoteHub. Defines commands/formats and the
    encode/decode functions shared with the sensor side."""

    def __init__(self, max_packet_size=MAX_PKT):
        self.commands = []
        self.modes = {}
        self.max_packet_size = max_packet_size

    def add_command(self, mode_name, to_hub_fmt="", from_hub_fmt="", command_type=CALLBACK):
        if to_hub_fmt == "repr" or from_hub_fmt == "repr":
            msg_size = self.max_packet_size
            num_args_from_hub = -1
            num_args_to_hub = -1
        else:
            size_to_hub_fmt = struct.calcsize(to_hub_fmt)
            size_from_hub_fmt = struct.calcsize(from_hub_fmt)
            msg_size = max(size_to_hub_fmt, size_from_hub_fmt)
            num_args_to_hub = len(
                struct.unpack(to_hub_fmt, bytearray(struct.calcsize(to_hub_fmt)))
            )
            num_args_from_hub = len(
                struct.unpack(from_hub_fmt, bytearray(struct.calcsize(from_hub_fmt)))
            )

        assert msg_size <= self.max_packet_size, "Payload exceeds maximum packet size"
        self.commands.append({
            NAME: mode_name,
            TO_HUB_FORMAT: to_hub_fmt,
            SIZE: msg_size,
            ARGS_TO_HUB: num_args_to_hub,
        })
        if command_type == CALLBACK:
            self.commands[-1][FROM_HUB_FORMAT] = from_hub_fmt
            self.commands[-1][ARGS_FROM_HUB] = num_args_from_hub

        self.modes[mode_name] = len(self.commands) - 1

    def decode(self, fmt, data):
        if fmt == "repr":
            clean = data.rstrip(b"\x00")
            return (eval(clean),) if clean else ("",)
        else:
            size = struct.calcsize(fmt)
            data = struct.unpack(fmt, data[:size])
        return data

    def encode(self, size, format, *argv):
        if format == "repr":
            s = bytes(repr(*argv), "UTF-8")
        else:
            s = struct.pack(format, *argv)
        assert len(s) <= size, "Payload exceeds maximum packet size"
        return s


class PUPRemoteHub(PUPRemote):
    """Communicate with a PUPRemoteSensor from a Pybricks hub. `port` is
    where the PUPRemoteSensor is connected (e.g. Port.D)."""

    def __init__(self, port, max_packet_size=MAX_PKT):
        super().__init__(max_packet_size)
        if isinstance(port, str):
            port = eval("Port." + port)
        if isinstance(port, int):
            port = eval("Port." + chr(64 + port))
        else:
            self.port = port
        try:
            self.pup_device = PUPDevice(port)
        except OSError:
            self.pup_device = None
            print("Check wiring and remote script. Unable to connect on ", self.port)
            raise

    def add_command(self, mode_name, to_hub_fmt="", from_hub_fmt="", command_type=CALLBACK):
        super().add_command(mode_name, to_hub_fmt, from_hub_fmt, command_type)
        # Check the newly added command against what the sensor side advertises.
        modes = self.pup_device.info()["modes"]
        n = len(self.commands) - 1
        assert len(self.commands) <= len(modes), "More commands than on remote side"
        assert mode_name == modes[n][0].rstrip(), (
            "Expected '{}' as mode {}, but got '{}'".format(modes[n][0].rstrip(), n, mode_name)
        )
        assert self.commands[-1][SIZE] == modes[n][1], (
            "Different parameter size than on remote side. Check formats."
        )

    def call(self, mode_name, *argv, wait_ms=0):
        """Call a remote function on the sensor side, wait_ms before
        reading the reply back. Must not be used from a multitask
        context (not supported here at all -- see the file header)."""
        assert not run_task(), "Use 'call_multitask' instead of 'call', with multiple start blocks or multitask blocks"

        mode = self.modes[mode_name]
        size = self.commands[mode][SIZE]

        if FROM_HUB_FORMAT in self.commands[mode]:
            num_args = self.commands[mode][ARGS_FROM_HUB]
            if num_args >= 0:
                assert len(argv) == num_args, (
                    "Expected {} argument(s) in call '{}'".format(num_args, mode_name)
                )
            self.pup_device.read(mode)
            payl = self.encode(size, self.commands[mode][FROM_HUB_FORMAT], *argv)
            self.pup_device.write(
                mode,
                [((i + 128) & 0xFF) - 128 for i in tuple(payl + b"\x00" * (size - len(payl)))],
            )
            wait(wait_ms)

        data = self.pup_device.read(mode)
        raw_data = bytes([b if b >= 0 else b + 256 for b in data])
        result = self.decode(self.commands[mode][TO_HUB_FORMAT], raw_data)
        return result[0] if len(result) == 1 else result


# --- this project's own code -------------------------------------------

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.tools import wait

hub = PrimeHub()

# to_hub_fmt/from_hub_fmt must match camera.py's add_command("mode", ...)
# exactly -- same name, same two format strings, on both sides. The reply
# is 8 shorts; f2..f7 mean different things depending on echoed_mode, per
# camera.py's own comment above its "mode" registration:
#   LINE (echoed_mode=0): ahead, angle, length, coverage, bg_code, unused
#   ZONE (echoed_mode=1): sphere_count, first_kind, first_bearing,
#                         first_dist_mm, green_found, red_found
camera = PUPRemoteHub(Port.D)
camera.add_command("mode", to_hub_fmt="hhhhhhhh", from_hub_fmt="b")

BACKGROUND_NAME = ["black", "white", "unclear"]
SPHERE_KIND_NAME = {-1: "none", 0: "dead", 1: "live"}

zone_mode = False  # local desired state -- starts in LINE mode

while True:
    pressed = hub.buttons.pressed()

    if Button.LEFT in pressed:
        zone_mode = False
    elif Button.RIGHT in pressed:
        zone_mode = True

    # wait_ms: small delay between writing the command and reading the
    # reply back, per PUPRemoteHub.call()'s own docstring suggestion
    # (roughly struct.calcsize(from_hub_fmt) * 1.5) -- without it there's
    # a real chance of reading back whatever the camera last had queued
    # before this call's write actually lands.
    echoed_mode, heartbeat, f2, f3, f4, f5, f6, f7 = camera.call(
        "mode", 1 if zone_mode else 0, wait_ms=5
    )

    # This is sent as a level every tick, not a one-shot event (design.md
    # Sec3's "levels, not verbs" reasoning for the whole hub1<->hub2 link
    # applies here too) -- if the camera hasn't caught this particular
    # write yet, echoed_mode will still show the OLD mode for a tick or
    # two, and the next resend will just try again on its own. Printing
    # that explicitly instead of only ever showing the current reply
    # means a genuinely slow switch is visible as "switching..." rather
    # than looking identical to a working one that just hasn't been
    # pressed differently yet.
    if echoed_mode != (1 if zone_mode else 0):
        print("(switching... camera hasn't caught the last change yet)")

    if echoed_mode == 0:
        ahead, angle, length, coverage, bg_code = f2, f3, f4, f5, f6
        bg_name = BACKGROUND_NAME[bg_code] if 0 <= bg_code <= 2 else "?"
        # tracking follows the opposite colour to the background, same
        # rule camera.py itself uses -- derived here rather than sent,
        # since it doesn't need its own field.
        label = "WHT" if bg_name == "black" else "BLK"
        print("%s ahead=%d angle=%d len=%d cov=%d%% | background: %s" %
              (label, ahead, angle, length, coverage, bg_name))
    else:
        count, kind, bearing, dist, green_found, red_found = f2, f3, f4, f5, f6, f7
        print("ZONE spheres=%d first=%s bearing=%ddeg dist=%dmm green=%s red=%s" %
              (count, SPHERE_KIND_NAME.get(kind, "?"), bearing, dist,
               "y" if green_found else "n", "y" if red_found else "n"))

    wait(50)
