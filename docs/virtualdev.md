# Recycle JTAG: Virtual Development

The virtual development environment creates pseudo-terminals on Linux to allow OpenOCD to think that it's communicating over a serial port to a Bus Pirate.

The main purpose of virtualdev is to allow rapid prototyping without the need for external hardware.

There are two ways to have it create the virtual serial port, and have OpenOCD connect to it:
1. [Automatically](#automatically) - Usually just works, but if another terminal opens up, the pseudo-terminal entry might change.
2. [Manually](#manually) - Can be useful if you want to persist the pseudo-terminal that's used.

## Automatically

Run `rjtag` without any arguments, and it will automatically attempt to create a pseudo-terminal, and report its location on stdout:

```
$ build/rjtag
Connect OpenOCD to: '/dev/pts/4'
```

You may now run:

```
openocd -f interface/buspirate.cfg -c 'buspirate port /dev/pts/4'
```

## Manually

You can manually create ptys by using `socat`:

```
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
2023/03/03 23:48:34 socat[2889] N PTY is /dev/pts/7
2023/03/03 23:48:34 socat[2889] N PTY is /dev/pts/8
```

The actual files in `/dev/pts/` will vary on your system.

Select one as the interface for `rjtag`, and the other for OpenOCD:

In one terminal:
```
build/rjtag /dev/pts/7
```

In another terminal:
```
openocd -f interface/buspirate.cfg -c 'buspirate port /dev/pts/8'
```

The advantage of this method, despite it being a bit more cumbersome is that the created pseudo-terminal entries persist and are constant as long as `socat` is running, no matter how many terminals are open.

I find that I open up and close terminals a lot, which means that the entries in `/dev/pts` constantly change, which means that the automatically assigned pseudo-terminal changes every now and then.