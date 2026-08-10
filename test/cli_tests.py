#!/usr/bin/env python3

import json
import pathlib
import socket
import subprocess
import sys
import tempfile
import time


def run(tracer, *args, check=True):
    result = subprocess.run(
        [tracer, *map(str, args)],
        check=False,
        capture_output=True,
        text=True,
    )
    if check and result.returncode != 0:
        raise AssertionError(
            f"command failed ({result.returncode}): {result.args}\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    return result


def json_lines(output):
    return [json.loads(line) for line in output.splitlines() if line.strip()]


def put_word(image, address, value):
    image[address] = value >> 8
    image[address + 1] = value & 0xFF


def rsp_frame(payload, checksum=None):
    encoded = payload.encode("ascii")
    if checksum is None:
        checksum = sum(encoded) & 0xFF
    return b"$" + encoded + f"#{checksum:02x}".encode("ascii")


def recv_exact(connection, length):
    data = bytearray()
    while len(data) < length:
        chunk = connection.recv(length - len(data))
        if not chunk:
            raise AssertionError("GDB connection closed unexpectedly")
        data.extend(chunk)
    return bytes(data)


def recv_rsp_reply(connection):
    assert recv_exact(connection, 1) == b"$"
    payload = bytearray()
    while True:
        value = recv_exact(connection, 1)
        if value == b"#":
            break
        payload.extend(value)
    checksum = int(recv_exact(connection, 2), 16)
    assert checksum == (sum(payload) & 0xFF)
    connection.sendall(b"+")
    return payload.decode("ascii")


def rsp_command(connection, payload):
    connection.sendall(rsp_frame(payload))
    assert recv_exact(connection, 1) == b"+"
    return recv_rsp_reply(connection)


def test_gdb_protocol(tracer, simple):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]

    process = subprocess.Popen(
        [tracer, f"--gdb={port}", "--quiet", simple],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    connection = None
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                connection = socket.create_connection(("127.0.0.1", port), 0.2)
                break
            except OSError:
                if process.poll() is not None:
                    break
                time.sleep(0.02)
        if connection is None:
            stdout, stderr = process.communicate(timeout=1)
            raise AssertionError(
                f"GDB server did not start\nstdout:\n{stdout}\nstderr:\n{stderr}"
            )

        with connection:
            connection.settimeout(2)

            # A bad checksum is rejected without dispatching the packet.
            connection.sendall(rsp_frame("?", checksum=0))
            assert recv_exact(connection, 1) == b"-"
            assert rsp_command(connection, "?") == "S05"

            registers = rsp_command(connection, "g")
            assert len(registers) == 16 * 8

            # Short writes fail and leave architectural state untouched.
            assert rsp_command(connection, "P0=12") == "E01"
            assert rsp_command(connection, "p0") == "00000000"
            assert rsp_command(connection, "G00000000") == "E01"
            assert rsp_command(connection, "p0") == "00000000"
            assert rsp_command(connection, "M10,2:aa") == "E01"
            assert rsp_command(connection, "m10,2") == "0000"

            # PC/WP writes are normalized and complete memory writes round-trip.
            assert rsp_command(connection, "P0=01000000") == "OK"
            assert rsp_command(connection, "p0") == "00000000"
            assert rsp_command(connection, "M10,2:a55a") == "OK"
            assert rsp_command(connection, "m10,2") == "a55a"

            assert rsp_command(connection, "D") == "OK"

        assert process.wait(timeout=2) == 0
    finally:
        if connection is not None:
            connection.close()
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=2)


def make_event_image(path):
    image = bytearray(0x304)
    put_word(image, 0x0004, 0x1100)  # Level-1 WP.
    put_word(image, 0x0006, 0x0300)  # Level-1 PC.
    put_word(image, 0x0200, 0x0300)  # LIMI >0001.
    put_word(image, 0x0202, 0x0001)
    put_word(image, 0x0204, 0x0340)  # IDLE.
    put_word(image, 0x0300, 0x0200)  # LI R0,>A55A.
    put_word(image, 0x0302, 0xA55A)
    path.write_bytes(image)


def make_loop_image(path):
    image = bytearray(0x304)
    put_word(image, 0x0004, 0x1100)
    put_word(image, 0x0006, 0x0300)
    put_word(image, 0x0200, 0x0300)  # LIMI >0001.
    put_word(image, 0x0202, 0x0001)
    put_word(image, 0x0204, 0x10FF)  # JMP $.
    put_word(image, 0x0300, 0x0200)  # LI R0,>5AA5.
    put_word(image, 0x0302, 0x5AA5)
    path.write_bytes(image)


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: cli_tests.py TRACER SIMPLE_BIN")

    tracer = sys.argv[1]
    simple = sys.argv[2]

    baseline = json_lines(
        run(
            tracer,
            "--summary",
            "--no-loop-detect",
            "--max-steps=4",
            simple,
        ).stdout
    )[-1]
    assert baseline["steps"] == 4
    assert baseline["clk"] == 48
    assert baseline["pc"] == "000A"

    trace = json_lines(
        run(tracer, "--quiet", "--max-steps=1", simple).stdout
    )
    assert len(trace) == 1
    assert trace[0]["step"] == 0
    assert trace[0]["pc"] == "0000"
    assert trace[0]["clk"] == 0
    assert trace[0]["r"][0] == "0000"

    waited = json_lines(
        run(
            tracer,
            "--summary",
            "--quiet",
            "--wait-states=2",
            "--max-steps=1",
            simple,
        ).stdout
    )[-1]
    assert waited["clk"] == 18  # LI: C=12, M=3, W=2.

    interrupted = json_lines(
        run(
            tracer,
            "--quiet",
            "--max-steps=3",
            "--irq=0@2",
            simple,
        ).stdout
    )
    assert [record["step"] for record in interrupted] == [0, 1, 2]
    assert interrupted[2]["pc"] == "1234"
    assert interrupted[2]["op"] == "INVL"
    assert interrupted[2]["clk"] == 50

    filtered = json_lines(
        run(
            tracer,
            "--summary",
            "--quiet",
            "--tracepoint=0x0000",
            "--tracepoint-max=1",
            simple,
        ).stdout
    )
    assert len(filtered) == 2
    assert filtered[0] == trace[0]
    assert filtered[1]["steps"] == 1
    assert filtered[1]["halt"] == "tracepoint-max"

    rejected = run(tracer, "--dump=0xFFFF:2", simple, check=False)
    assert rejected.returncode != 0
    assert "Invalid dump spec" in rejected.stderr

    odd_dump = run(
        tracer,
        "--summary",
        "--quiet",
        "--max-steps=0",
        "--dump=0x0001:3",
        simple,
    )
    assert "0001: 00 12 34" in odd_dump.stderr

    full_dump = run(
        tracer,
        "--summary",
        "--quiet",
        "--max-steps=0",
        "--dump=0x0000:65536",
        simple,
    )
    assert "(65536 bytes)" in full_dump.stderr
    assert "FFF0:" in full_dump.stderr

    with tempfile.TemporaryDirectory() as directory:
        directory = pathlib.Path(directory)

        idle_image = directory / "idle_timer.bin"
        make_event_image(idle_image)
        idle = json_lines(
            run(
                tracer,
                "--summary",
                "--quiet",
                "--entry=0x0200",
                "--wp=0x1000",
                "--max-steps=3",
                "--timer=1:50",
                idle_image,
            ).stdout
        )[-1]
        assert idle["steps"] == 3
        assert idle["clk"] == 84
        assert idle["pc"] == "0304"
        assert idle["wp"] == "1100"
        assert idle["r"][0] == "A55A"

        loop_image = directory / "future_irq.bin"
        make_loop_image(loop_image)
        loop = json_lines(
            run(
                tracer,
                "--summary",
                "--quiet",
                "--entry=0x0200",
                "--wp=0x1000",
                "--max-steps=6",
                "--irq=1@5",
                loop_image,
            ).stdout
        )[-1]
        assert loop["steps"] == 6
        assert loop["halt"] == "max"
        assert loop["pc"] == "0304"
        assert loop["r"][0] == "5AA5"

    test_gdb_protocol(tracer, simple)

    print("CLI integration checks passed")


if __name__ == "__main__":
    main()
