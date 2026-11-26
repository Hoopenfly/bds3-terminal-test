# -*- coding: utf-8 -*-
"""bd3_test 协议层与判定状态机自检(不依赖串口与界面)."""

import os
import sys
import unittest

sys.dont_write_bytecode = True  #不生成 __pycache__
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

import bd3_test as bd


class TestProtocol(unittest.TestCase):
    def test_checksum_sample(self):
        self.assertEqual(bd.xor_checksum("CCTCQ,1881210,2,1,2,0123456789ABCDEF,0"), 0x72)
        self.assertEqual(bd.xor_checksum("BDFKI,085051,TCQ,Y,0,0"), 0x78)
        self.assertEqual(bd.xor_checksum("BDTCI,1881210,1881210,2,165052,2,0,0123456789ABCDEF"), 0x47)

    def test_build_cctcq_matches_sample(self):
        frame = bd.build_cctcq("1881210", "0123456789ABCDEF")
        self.assertEqual(frame, b"$CCTCQ,1881210,2,1,2,0123456789ABCDEF,0*72\r\n")

    def test_build_icr(self):
        self.assertEqual(bd.build_icr_query(), b"$CCICR,0,00*68\r\n")

    def test_parse_fki(self):
        s = bd.parse_sentence(b"$BDFKI,085051,TCQ,Y,0,0*78\r\n")
        self.assertIsNotNone(s)
        self.assertTrue(s.checksum_ok)
        self.assertEqual(s.type_id, "BDFKI")
        self.assertEqual(s.field(2), "TCQ")
        self.assertEqual(s.field(3), "Y")
        self.assertEqual(s.field(4), "0")

    def test_parse_tci(self):
        s = bd.parse_sentence(b"$BDTCI,1881210,1881210,2,165052,2,0,0123456789ABCDEF*47")
        self.assertTrue(s.checksum_ok)
        self.assertEqual(s.field(1), "1881210")
        self.assertEqual(s.field(2), "1881210")
        self.assertEqual(s.field(7), "0123456789ABCDEF")

    def test_parse_bad_checksum(self):
        s = bd.parse_sentence(b"$BDFKI,085051,TCQ,Y,0,0*00")
        self.assertIsNotNone(s)
        self.assertFalse(s.checksum_ok)

    def test_parse_garbage(self):
        self.assertIsNone(bd.parse_sentence(b"hello world"))


class TestSession(unittest.TestCase):
    ADDR = "1881210"
    DATA = "0123456789ABCDEF"

    def make(self, wait_s=60.0):
        return bd.TestSession(self.ADDR, self.DATA, wait_s)

    def fki(self, status, reason="0", remain="0", t="085051"):
        return bd.parse_sentence(bd.build_sentence(
            "BDFKI,%s,TCQ,%s,%s,%s" % (t, status, reason, remain)))

    def tci(self, tx=None, rx=None, data=None, coding="2"):
        return bd.parse_sentence(bd.build_sentence(
            "BDTCI,%s,%s,2,165052,%s,0,%s" % (tx or self.ADDR, rx or self.ADDR,
                                              coding, data or self.DATA)))

    def test_success_path(self):
        s = self.make()
        rec, _frame, _ = s.start_tx(0.0)
        s.on_sentence(self.fki("Y"), 0.5)
        s.on_sentence(self.tci(), 1.2)
        self.assertEqual(rec.result, "success")
        self.assertEqual((s.sent_count, s.success_count, s.fail_count), (1, 1, 0))

    def test_retry_n_then_y(self):
        s = self.make()
        rec, _frame, _ = s.start_tx(0.0)
        s.on_sentence(self.fki("N", "1", "10"), 0.2)
        self.assertFalse(rec.finalized)  # 频率未到, 继续等重试
        s.on_sentence(self.fki("Y"), 11.0)
        s.on_sentence(self.tci(), 11.8)
        self.assertEqual(rec.result, "success")
        self.assertEqual(s.success_count, 1)

    def test_n_other_reason_fails_fast(self):
        s = self.make()
        rec, _frame, _ = s.start_tx(0.0)
        s.on_sentence(self.fki("N", "6", "0"), 0.2)
        self.assertTrue(rec.finalized)
        self.assertEqual(rec.result, "fail")
        self.assertIn("权限不足", rec.fail_reason)

    def test_timeout_without_tci(self):
        s = self.make(wait_s=10.0)
        rec, _frame, _ = s.start_tx(0.0)
        s.on_sentence(self.fki("Y"), 0.5)
        s.tick(9.9)
        self.assertFalse(rec.finalized)
        s.tick(10.0)
        self.assertTrue(rec.finalized)
        self.assertEqual(rec.result, "fail")
        self.assertIn("未收到匹配的回环报文", rec.fail_reason)

    def test_timeout_without_any_fki(self):
        s = self.make(wait_s=10.0)
        rec, _frame, _ = s.start_tx(0.0)
        s.tick(10.0)
        self.assertEqual(rec.result, "fail")
        self.assertIn("未收到 FKI", rec.fail_reason)

    def test_tci_mismatch_then_timeout(self):
        s = self.make(wait_s=10.0)
        rec, _frame, _ = s.start_tx(0.0)
        s.on_sentence(self.fki("Y"), 0.5)
        s.on_sentence(self.tci(data="FFFFFFFF"), 1.0)
        self.assertFalse(rec.finalized)
        s.tick(10.0)
        self.assertEqual(rec.result, "fail")
        self.assertIn("不匹配", rec.fail_reason)

    def test_stop_marks_interrupted(self):
        s = self.make()
        rec, _frame, _ = s.start_tx(0.0)
        s.stop()
        self.assertEqual(rec.result, "interrupted")
        self.assertEqual((s.sent_count, s.success_count, s.fail_count), (1, 0, 0))

    def test_second_send_finalizes_previous(self):
        s = self.make(wait_s=10.0)
        rec1, _f1, _ = s.start_tx(0.0)
        rec2, _f2, events = s.start_tx(12.0)
        self.assertEqual(rec1.result, "fail")
        self.assertEqual(rec2.seq, 2)
        self.assertEqual(s.sent_count, 2)
        self.assertTrue(events)

    def test_success_rate(self):
        s = self.make()
        rec, _f, _ = s.start_tx(0.0)
        s.on_sentence(self.fki("Y"), 0.5)
        s.on_sentence(self.tci(), 1.0)
        self.assertAlmostEqual(s.success_rate, 100.0)
        s2 = self.make(wait_s=1.0)
        s2.start_tx(0.0)
        s2.tick(1.0)
        self.assertAlmostEqual(s2.success_rate, 0.0)


class TestFinishCallback(unittest.TestCase):
    """判定回调: 每个事务结束必须恰好回调一次, 供界面追写记录文件."""

    def setUp(self):
        self.calls = []

    def make(self, wait_s=10.0):
        return bd.TestSession("1881210", "0123456789ABCDEF", wait_s,
                              on_finish=lambda *args: self.calls.append(args))

    def fki(self, status, reason="0"):
        return bd.parse_sentence(bd.build_sentence(
            "BDFKI,085051,TCQ,%s,%s,0" % (status, reason)))

    def tci(self):
        return bd.parse_sentence(bd.build_sentence(
            "BDTCI,1881210,1881210,2,165052,2,0,0123456789ABCDEF"))

    def test_callback_on_success(self):
        s = self.make()
        s.start_tx(0.0)
        s.on_sentence(self.fki("Y"), 0.5)
        s.on_sentence(self.tci(), 1.0)
        self.assertEqual(len(self.calls), 1)
        rec, result, reason, now = self.calls[0]
        self.assertEqual(result, "success")
        self.assertEqual(now, 1.0)

    def test_callback_on_fail(self):
        s = self.make()
        s.start_tx(0.0)
        s.tick(10.0)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.calls[0][1], "fail")

    def test_callback_on_interrupt(self):
        s = self.make()
        s.start_tx(0.0)
        s.stop()
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.calls[0][1], "interrupted")

    def test_no_duplicate_callback(self):
        s = self.make()
        s.start_tx(0.0)
        s.tick(10.0)
        s.tick(20.0)
        s.stop()
        self.assertEqual(len(self.calls), 1)


class TestPosition(unittest.TestCase):
    """定位测试: RNS/RMO 指令构造与 RMC 判定."""

    def rmc(self, status="A", lat="3140.12345", lon="11848.56789",
            mode="A", name="GNRMC"):
        body = "%s,085051.00,%s,%s,N,%s,E,0.0,0.0,260325,,,%s" % (
            name, status, lat, lon, mode)
        return bd.parse_sentence(bd.build_sentence(body))

    def test_build_ccrns(self):
        self.assertEqual(bd.build_ccrns_rmc(), b"$CCRNS,0,0,0,0,1,0,8*5A\r\n")

    def test_build_rmo_rmc_freq(self):
        self.assertEqual(bd.build_rmc_freq(3), b"$CCRMO,RMC,2,3*21\r\n")
        self.assertEqual(bd.build_rmc_freq(1), b"$CCRMO,RMC,2,1*23\r\n")
        self.assertEqual(bd.build_rmc_freq(0), b"$CCRMO,RMC,2,0*22\r\n")

    def test_build_rmo_pwi(self):
        self.assertEqual(bd.build_pwi_off(), b"$CCRMO,PWI,1,0*33\r\n")
        self.assertEqual(bd.build_pwi_once(), b"$CCRMO,PWI,2,0*30\r\n")

    def test_parse_rmc(self):
        rec = bd.parse_rmc(self.rmc())
        self.assertEqual(rec.status, "A")
        self.assertEqual(rec.lat, "3140.12345")
        self.assertEqual(rec.lon, "11848.56789")
        self.assertEqual(rec.mode, "A")
        self.assertIsNone(bd.parse_rmc(bd.parse_sentence(
            bd.build_sentence("BDFKI,085051,TCQ,Y,0,0"))))

    def test_valid(self):
        calls = []
        ps = bd.PositionSession(on_finish=lambda *a: calls.append(a))
        ps.on_rmc(self.rmc(), 1.0)
        self.assertEqual((ps.total, ps.valid_count, ps.invalid_count), (1, 1, 0))
        self.assertAlmostEqual(ps.success_rate, 100.0)
        self.assertTrue(ps.last.valid)
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0][1])

    def test_status_v(self):
        ps = bd.PositionSession()
        ps.on_rmc(self.rmc(status="V", lat="", lon=""), 1.0)
        self.assertEqual((ps.total, ps.valid_count, ps.invalid_count), (1, 0, 1))
        self.assertIn("定位状态", ps.last.fail_reason)
        self.assertAlmostEqual(ps.success_rate, 0.0)

    def test_empty_coord(self):
        ps = bd.PositionSession()
        ps.on_rmc(self.rmc(status="A", lat="", lon=""), 1.0)
        self.assertEqual(ps.invalid_count, 1)
        self.assertIn("经纬度", ps.last.fail_reason)

    def test_mode_n(self):
        ps = bd.PositionSession()
        ps.on_rmc(self.rmc(mode="N"), 1.0)
        self.assertEqual(ps.invalid_count, 1)
        self.assertIn("模式", ps.last.fail_reason)

    def test_bdrmc_and_rate(self):
        ps = bd.PositionSession()
        ps.on_rmc(self.rmc(name="BDRMC"), 1.0)
        ps.on_rmc(self.rmc(status="V", lat="", lon=""), 2.0)
        ps.on_rmc(self.rmc(), 3.0)
        self.assertEqual((ps.total, ps.valid_count, ps.invalid_count), (3, 2, 1))
        self.assertAlmostEqual(ps.success_rate, 66.6666, places=3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
