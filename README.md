## ublox-core
An embedded hal (no_std) driver for communicating with Ublox position 
devices (M8 or later)

## Status

This is work-in-progress

- [x] Basic support for USART (serial) reads
- [x] library builds ok (dev builds only currently)
- [ ] release library builds ok (need to remove debug scaffolding)
- [ ] Parsing of a few key message types:
    UBX-NAV-PVT, UBX-NAV-DOP, UBX-MON-HW
- [ ] USB support
- [ ] SPI support
- [ ] I2C support


## Examples

Build example for eg Durandal stm32h743 board target:
```
cargo build --example serial --target thumbv7em-none-eabihf 
```


## License

BSD-3-Clause, see `LICENSE` file.
 
