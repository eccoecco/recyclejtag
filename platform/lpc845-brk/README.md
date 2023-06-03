# LPC845-BRK Breakout Board for RJtag

See [here](../../docs/lpc845-brk.md) for more in-depth documentation.

## Manual Debugging

To manually feed data in: 

```
stty -F /dev/ttyACM0 raw && stty -F /dev/ttyACM0 -echo -echoe -echok && stty -F /dev/ttyACM0 115200
```

On one terminal:
```
cat /dev/ttyACM0 | hexdump -v -e '/1 "%02X "' -e '/1 "%c\n"'
```

On another:
```
printf '\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0' > /dev/ttyACM0
```

## OpenOCD invocation

### Linux

openocd -f interface/buspirate.cfg -c 'buspirate port /dev/ttyACM1'  -c 'transport select jtag'

### Windows

When everything is working, you can just do:

```
openocd.exe -f interface/buspirate.cfg -c 'buspirate port COM4'  -c 'transport select jtag'
```

But if you need to debug, e.g. access raw serial data to see what's going on:

```
openocd.exe -f interface/buspirate.cfg -c 'buspirate port COM4'  -c 'transport select jtag' -d3
```

## SVD for Cortex Debug

The SVD file allows the Cortex Debug extension to view peripheral registers.

I grabbed a copy from [here](https://github.com/lpc-rs/lpc-pac/blob/master/lpc845/lpc845.svd).
