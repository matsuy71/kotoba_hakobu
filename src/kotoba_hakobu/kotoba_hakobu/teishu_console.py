"""亭主の操作盤：指示を読み取り、聞き返し・復唱で確かめてから、お出しするサーバーに注文し、結果を報告する"""
import json
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from kotoba_hakobu_interfaces.action import ServeTea
from kotoba_hakobu.understanding import understand

BOWL_JA = {"red": "赤楽のお茶碗", "black": "黒楽のお茶碗"}
SEAT_JA = {"shokyaku": "お正客様", "jikyaku": "お次客様", "sankyaku": "お三客様"}
ASK = {"bowl": "どちらのお茶碗にいたしましょう？（赤楽・黒楽）",
       "seat": "どなたにお出しいたしましょう？（お正客様・お次客様・お三客様）"}
LOG_DIR = "/workspace/results"


def is_yes(text):
    """「はい」「お願いします」なら True。それ以外は、すべて取りやめとみなす。"""
    return text.startswith("はい") or text.startswith("お願い") or text.lower() in ("y", "yes")


class TeishuConsole(Node):
    """亭主の操作盤のノード。お出しするサーバー（serve_tea）に注文を出す。"""

    def __init__(self):
        super().__init__("teishu_console")
        self.client = ActionClient(self, ServeTea, "serve_tea")
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = f"{LOG_DIR}/chakai_log_{datetime.now():%Y%m%d_%H%M%S}.jsonl"

    def ask_missing(self, order, key, record):
        """足りない項目（key）を聞き返して埋める。2回聞いても分からなければ False を返す。"""
        for _ in range(2):
            reply_text = input(f"ロボット：{ASK[key]}\n亭主：").strip()
            reply = understand(reply_text)
            record["replies"].append({"text": reply_text, "understanding": reply})
            for k in ("bowl", "seat"):          # 返事に、もう一方の項目が入っていれば、それも使う
                if reply[k] != "unknown":
                    order[k] = reply[k]
            if order[key] != "unknown":
                return True
            if reply["intent"] == "other":
                return False
            print("ロボット：申し訳ございません。もう一度お聞かせください。")
        return False

    def serve(self, bowl, seat):
        """サーバーに注文を出し、途中経過を表示しながら、結果を待って返す。"""
        if not self.client.wait_for_server(timeout_sec=5.0):
            return None
        goal = ServeTea.Goal()
        goal.bowl, goal.seat = bowl, seat
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        while not future.done():
            time.sleep(0.1)
        handle = future.result()
        if not handle.accepted:
            return None
        result_future = handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)
        return result_future.result().result

    def on_feedback(self, message):
        feedback = message.feedback
        print(f"　　（{feedback.step_index}/{feedback.step_count}）{feedback.step}")

    def take_order(self, text):
        """亭主の指示を1つ受けて、確かめてからお出しし、報告して、記録を書き足す。"""
        order = understand(text)
        record = {"time": datetime.now().isoformat(), "text": text,
                  "understanding": dict(order), "replies": []}
        sources = [order["source"]]

        if order["intent"] == "other":
            print("ロボット：申し訳ございません。お茶をお出しすること以外は、承れません。")
            record["outcome"] = "お断り（お茶をお出しする頼みではない）"
        else:
            ok = True
            for key in ("bowl", "seat"):
                if ok and order[key] == "unknown":
                    ok = self.ask_missing(order, key, record)
            sources += [r["understanding"]["source"] for r in record["replies"]]
            if not ok:
                print("ロボット：かしこまりました。このたびは控えさせていただきます。")
                record["outcome"] = "取りやめ（聞き返しても分からなかった）"
            else:
                note = "（予備の方法で読み取りました。よくお確かめください）" if "keyword" in sources else ""
                answer = input(f"ロボット：{BOWL_JA[order['bowl']]}を、{SEAT_JA[order['seat']]}に"
                               f"お出しいたします。よろしいですか？{note}（はい／いいえ）\n亭主：").strip()
                record["confirm"] = answer
                if not is_yes(answer):
                    print("ロボット：かしこまりました。取りやめます。")
                    record["outcome"] = "取りやめ（確認で「はい」がなかった）"
                else:
                    record["order"] = {"bowl": order["bowl"], "seat": order["seat"]}
                    result = self.serve(order["bowl"], order["seat"])
                    if result is None:
                        print("ロボット：申し訳ございません。お出しする係に連絡がつきません。")
                        record["outcome"] = "失敗（サーバーにつながらない）"
                    else:
                        record["result"] = {"success": result.success, "message": result.message,
                                            "failed_step": result.failed_step,
                                            "placement_error_m": result.placement_error_m,
                                            "max_tilt_deg": result.max_tilt_deg,
                                            "elapsed_sec": result.elapsed_sec}
                        if result.success:
                            print(f"ロボット：{BOWL_JA[order['bowl']]}を、{SEAT_JA[order['seat']]}に"
                                  f"お出しいたしました（ずれ {result.placement_error_m * 1000:.1f}mm・"
                                  f"傾き {result.max_tilt_deg:.1f}度・{result.elapsed_sec:.0f}秒）")
                            record["outcome"] = "成功"
                        else:
                            print(f"ロボット：申し訳ございません。{result.message}")
                            record["outcome"] = f"失敗（{result.failed_step}）"

        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")


def main():
    rclpy.init()
    node = TeishuConsole()
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    print("亭主の操作盤です。ご指示をどうぞ（終わるときは「おわり」）")
    try:
        while True:
            text = input("亭主：").strip()
            if text in ("おわり", "終わり"):
                break
            if text:
                node.take_order(text)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        print(f"記録：{node.log_path}")
        rclpy.shutdown()
        thread.join()


if __name__ == "__main__":
    main()