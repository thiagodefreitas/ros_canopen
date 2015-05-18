#!/usr/bin/env python3

PDOs={
"RPDO1": [1, ('6040',     "control_word",                 2), ('6060',     "op_mode",                      1)],
"RPDO3": [1, ('607A',     "target_position",              4), ('60FF',     "target_profiled_velocity",     4)],
"RPDO4": [1, ('60C1sub1', "target_interpolated_position", 4)],

"TPDO1": [1, ('6041',     "status_word",                  2), ('6061',     "op_mode_display",              1), ('2041',     "timestamp",     4)],
"TPDO3": [1, ('6064',     "actual_position",              4), ('606C',     "actual_velocity",              4)],
}

if __name__ == "__main__":
    import pdo, sys

    fname = sys.argv[1]
    pdo.patch_all(fname, fname, PDOs)
