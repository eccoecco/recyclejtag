# Recycle JTAG: Virtual Development

The virtual development environment creates pseudo-terminals on Linux to allow OpenOCD to think that it's communicating over a serial-ish port to a Bus Pirate.

You can manually create ptys by using `socat`:

```
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
2023/03/03 23:48:34 socat[2889] N PTY is /dev/pts/7
2023/03/03 23:48:34 socat[2889] N PTY is /dev/pts/8
```

The actual files in `/dev/pts/` will vary on your system.

You may optionally pass one of the pseudo terminals as the first argument to `rjtag` to establish a connection:
```
build/rjtag /dev/pts/8
```

Alternatively, calling `rjtag` without any arguments will have it automatically create its own pseudo-terminal, and report it to stdout.