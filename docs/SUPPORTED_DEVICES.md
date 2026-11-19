# Supported devices

Repository scope and runtime compatibility are deliberately separate. A
sensor is supported only after its complete capture-to-match path has been
implemented and exercised on hardware.

| Device | Transport | Capture | Calibration | Matcher | Enrollment | Verify and learning | Hardware status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Goodix GDIX51C0 | ACPI-enumerated SPI through spidev, GPIO reset and IRQ, TLS-PSK | 80×64 packed 12-bit capture decoded to raw 16-bit Chicago input | Sensor-bound 16-byte OTP id plus `0x224b0` Chicago payload and raw ImageBase | Native Chicago type-24 pipeline | Official 12 accepted-stage policy; rejected position samples remain retained | Three-attempt identify, capacity-50 `templateStudy`, fprintd persistence | Enrollment, match/no-match, learning to capacity, replacement, persistence, daemon restart, post-restart match, and suspend/resume cold-open refresh verified |
| Goodix GXFP51A0 (HC32F460, fw `GF_HC460SEC_APP_14103`) | As GDIX51C0, with a level-triggered IRQ and per-board GPIO lines (`config/gdix51c0-boards.tsv`); TLS key is the factory PMK, read once through the `F4` CRC check | 80 rows of 88 raw12 samples, first 64 real | 256-byte `0x90` config; DAC set by register writes | Same as GDIX51C0 | Same as GDIX51C0 | Same as GDIX51C0 | Enrollment, verification and LightDM login verified on Huawei BOHB-WAX9 |

## Reference implementations are not supported devices

The repository may contain ignored local checkouts of other Goodix drivers.
They are evidence for choosing component boundaries, not devices shipped by
this project:

- GoodixTLS 5110 demonstrates the same outer/command framing and raw12 packing,
  but uses USB, an 88×80 scan/crop path and a different TLS integration.
- The reference Goodix 53x5 implementation demonstrates the same inner command
  envelope, raw12 packing and CRC-32/MPEG-2 primitive. It uses USB chunking,
  108×88 captures, a different GTLS/GEA path and SIGFM, so it does not establish
  a shared GDIX51C0 transport, calibration or matcher.

Adding another row requires an independently verified device identity,
transport and initialization trace, capture format, calibration contract,
matcher/template evidence, and hardware regression set.
