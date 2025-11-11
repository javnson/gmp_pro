import unittest
import symengine as se

# 假设这个测试文件与 mna_utils.py 在同一个目录下
from mna_utils import parse_value

class TestParseValue(unittest.TestCase):
    """
    针对 mna_utils.py 中 parse_value 函数的单元测试。
    """

    def assertApproxEqual(self, val1, val2, msg=None):
        """
        自定义断言，用于比较两个 symengine Rational 或浮点数是否近似相等。
        """
        self.assertTrue(se.sympify(abs(val1 - val2)) < 1e-9, msg)

    def test_plain_numbers(self):
        """测试不带单位的普通数字和科学记数法。"""
        self.assertEqual(parse_value("100"), se.Rational(100))
        self.assertEqual(parse_value("0.05"), se.Rational("0.05"))
        self.assertEqual(parse_value("1.23e-4"), se.Rational("0.000123"))
        self.assertEqual(parse_value("1e3"), se.Rational(1000))

    def test_case_insensitivity(self):
        """测试单位后缀的大小写不敏感性。"""
        self.assertEqual(parse_value("1k"), se.Rational(1000))
        self.assertEqual(parse_value("1K"), se.Rational(1000))
        self.assertEqual(parse_value("5m"), se.Rational("0.005"))
        self.assertEqual(parse_value("5M"), se.Rational("0.005"))
        self.assertEqual(parse_value("2u"), se.Rational("0.000002"))
        self.assertEqual(parse_value("2U"), se.Rational("0.000002"))

    def test_standard_suffixes(self):
        """测试所有标准的SPICE单位后缀。"""
        self.assertApproxEqual(parse_value("3T"), se.Rational(3e12))
        self.assertApproxEqual(parse_value("2G"), se.Rational(2e9))
        self.assertApproxEqual(parse_value("1.5K"), se.Rational(1500))
        self.assertApproxEqual(parse_value("50M"), se.Rational("0.05")) # 关键测试
        self.assertApproxEqual(parse_value("220U"), se.Rational("0.00022"))
        self.assertApproxEqual(parse_value("10N"), se.Rational(1e-8))
        self.assertApproxEqual(parse_value("4.7P"), se.Rational(4.7e-12))
        self.assertApproxEqual(parse_value("100F"), se.Rational(1e-13))

    def test_mega_suffix(self):
        """测试 'MEG' 后缀是否被正确处理。"""
        self.assertEqual(parse_value("2MEG"), se.Rational(2000000))
        self.assertEqual(parse_value("2.5MEG"), se.Rational("2500000"))
        # 测试 'MEG' 与 'M' 的优先级
        self.assertEqual(parse_value("10MEG"), se.Rational(10000000))

    def test_symbolic_values(self):
        """测试无法解析的字符串是否返回符号。"""
        self.assertEqual(parse_value("SYMBOLIC"), se.Symbol("SYMBOLIC"))
        self.assertEqual(parse_value("R_D1"), se.Symbol("R_D1"))
        # 这是一个无效的数字格式，应该被当作符号
        self.assertEqual(parse_value("1K2"), se.Symbol("1K2"))

    def test_edge_cases(self):
        """测试一些边界情况。"""
        # 数字和单位之间有空格的情况，当前设计会将其视为符号
        self.assertEqual(parse_value("10 K"), se.Symbol("10 K"))
        # 只有单位的情况
        self.assertEqual(parse_value("K"), se.Symbol("K"))


if __name__ == '__main__':
    print("Verifying functionality of mna_utils.py...")
    unittest.main(verbosity=2)
