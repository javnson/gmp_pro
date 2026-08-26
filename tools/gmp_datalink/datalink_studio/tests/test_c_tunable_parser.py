import unittest

from c_tunable_parser import parse_c_tunable_dictionary, strip_c_comments


EXAMPLE_DICTIONARY = r'''
// Tunable Dictionary
const gmp_param_item_t dict_m1[] = {
    // address,                         type,               permission,        name
    {&cia402_sm.current_cmd,            GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW, NULL},
    {&cia402_sm.current_state,          GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RO, NULL},
    {&protection.error_code,            GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW, NULL},
    {&mtr_ctrl.udc,                     GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.idq_ref.dat[phase_d],    GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&mtr_ctrl.idq_ref.dat[phase_q],    GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&mtr_ctrl.idq0.dat[phase_d],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.idq0.dat[phase_q],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.iuvw.dat[phase_U],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.iuvw.dat[phase_V],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.iuvw.dat[phase_W],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.vdq_ref.dat[phase_d],    GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&mtr_ctrl.vdq_ref.dat[phase_q],    GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&mech_ctrl.target_velocity,        GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&spd_enc.encif.speed,              GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
};
'''


class CTunableParserTests(unittest.TestCase):
    def test_current_four_field_null_name_dictionary(self) -> None:
        entries = parse_c_tunable_dictionary(EXAMPLE_DICTIONARY)
        self.assertEqual(len(entries), 15)
        self.assertEqual(entries[0]["name"], "cia402_sm.current_cmd")
        self.assertEqual(entries[4]["name"], "mtr_ctrl.idq_ref.dat[phase_d]")
        self.assertEqual(entries[-1]["name"], "spd_enc.encif.speed")
        self.assertEqual(entries[0]["type"], "GMP_PARAM_TYPE_U16")
        self.assertEqual(entries[-1]["perm"], "GMP_PARAM_PERM_RO")

    def test_named_current_and_legacy_entries(self) -> None:
        entries = parse_c_tunable_dictionary(r'''
            const gmp_param_item_t dictionary[] = {
                { &motor.kp, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, "Current-loop Kp" },
                { &motor.ki, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW },
                { (void *)&status.word, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RO,
                  "Status // word", "bitfield" },
                { &motor.empty, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, "" },
            };
        ''')
        self.assertEqual([item["name"] for item in entries], [
            "Current-loop Kp", "motor.ki", "Status // word", "motor.empty"
        ])
        self.assertEqual(entries[2]["unit"], "bitfield")

    def test_comments_do_not_damage_string_literals(self) -> None:
        source = '"http://target/*name*/" /* remove */ // remove too\nnext'
        self.assertEqual(strip_c_comments(source), '"http://target/*name*/"  \nnext')


if __name__ == "__main__":
    unittest.main()
