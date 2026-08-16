# canopen_eds_cc

`canopen_eds_cc.py` converts the object sections of a CiA 306-style EDS into a
GMP RB-tree object dictionary header and source file. It uses only the Python standard
library and performs deterministic generation.

Supported fields are `ParameterName`, `DataType`, `AccessType`, `DefaultValue` or
`ParameterValue`, `PDOMapping`, and `DataLength` for strings/domains. Sections use
`[1234]` or `[1234sub5]`. The GMP extension `GMPStorage=pointer|value` can override
the command-line default for one object. Integer defaults may contain `$NODEID` plus or
minus numeric constants.

The compiler rejects duplicate keys, unsupported types/access modes, malformed integer
expressions, raw defaults longer than `DataLength`, and inline values larger than eight
CAN octets. It does not attempt to implement the complete CiA 306 expression grammar,
compact sub-object notation, or vendor-specific data types.

Example:

```powershell
python canopen_eds_cc.py device.eds --output-dir generated --name device_od --storage pointer --node-id 7
python -m unittest discover -s tests -v
```

Generated pointer-storage variables are externally visible for application binding and
debugging. Generated raw/string/domain storage uses one `uint16_t` logical cell per
CANopen wire octet so it also compiles on C28x; only bits 7:0 are significant. Generated
public declarations include Doxygen descriptions derived from the EDS parameter names.
Call the generated `<name>_init()` exactly once for its static entry array, then use the
regular OD/SDO/PDO APIs.
