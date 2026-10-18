"""亭主の操作盤：指示を読み取り、聞き返し・復唱で確かめてから、お正客様から順にお出しし、結果を報告する"""
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
from kotoba_hakobu.understanding import BOWL_WORDS, SEAT_WORDS, find_one, understand, understand_orders

BOWL_JA = {"red": "赤楽のお茶碗", "black": "黒楽のお茶碗"}
SEAT_JA = {"shokyaku": "お正客様", "jikyaku": "お次客様", "sankyaku": "お三客様"}
SEAT_ORDER = ["shokyaku", "jikyaku", "sankyaku"]     # お出しする順番（いつもお正客様から）
MAX_ASK = 2                                          # 1つの項目を聞き返す回数の上限
LOG_DIR = "/workspace/results"


def is_yes(text):
    """「はい」「お願いします」なら True。それ以外は、すべて取りやめとみなす。"""
    return text.startswith("はい") or text.startswith("お願い") or text.lower() in ("y", "yes")


def seat_rank(order):
    """お出しする順番での位置。席が分からない注文は、いちばん後ろにする。"""
    return SEAT_ORDER.index(order["seat"]) if order["seat"] in SEAT_ORDER else len(SEAT_ORDER)


def other_bowl(bowl):
    """茶碗は2つなので、片方が決まれば、もう片方は残りの茶碗になる。"""
    return "black" if bowl == "red" else "red"


class TeishuConsole(Node):
    """亭主の操作盤のノード。お出しするサーバー（serve_tea）に、注文を1つずつ出す。"""

    def __init__(self):
        super().__init__("teishu_console")
        self.client = ActionClient(self, ServeTea, "serve_tea")
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = f"{LOG_DIR}/chakai_log_{datetime.now():%Y%m%d_%H%M%S}.jsonl"

    # ===== 聞き返し =====
    def ask(self, question, record):
        """1つ質問して、返事を読み取る。
        ①「黒楽」「お三客様」のような短い返事は、言葉で読む（候補が1つだけのとき）。
        ② 言葉が見つからないときは、質問と一緒に Gemini に読ませる（返事だけでは、頼みではないと読まれることがあるため）。"""
        reply_text = input(f"ロボット：{question}\n亭主：").strip()
        bowl, seat = find_one(reply_text, BOWL_WORDS), find_one(reply_text, SEAT_WORDS)
        if bowl != "unknown" or seat != "unknown":
            reply = {"bowl": bowl, "seat": seat, "intent": "serve", "source": "reply-word",
                     "model": None, "seconds": 0.0}
        else:
            reply = understand(f"（「{question}」への亭主の返事）{reply_text}")
        record["replies"].append({"question": question, "text": reply_text, "understanding": reply})
        return reply

    def ask_bowl(self, order, record):
        """茶碗を聞き返す。返事に席も入っていれば、それも使う。分からなければ False。"""
        if order["seat"] != "unknown":
            question = f"どちらのお茶碗を、{SEAT_JA[order['seat']]}にお出しいたしましょう？（赤楽・黒楽）"
        else:
            question = "どちらのお茶碗にいたしましょう？（赤楽・黒楽）"
        for _ in range(MAX_ASK):
            reply = self.ask(question, record)
            if reply["bowl"] != "unknown":
                order["bowl"] = reply["bowl"]
                if order["seat"] == "unknown" and reply["seat"] != "unknown":
                    order["seat"] = reply["seat"]
                return True
            if reply["intent"] == "other":
                return False
            print("ロボット：申し訳ございません。もう一度お聞かせください。")
        return False

    def ask_seat(self, order, record):
        """席を聞き返す。分からなければ False。"""
        question = f"{BOWL_JA[order['bowl']]}を、どなたにお出しいたしましょう？（お正客様・お次客様・お三客様）"
        for _ in range(MAX_ASK):
            reply = self.ask(question, record)
            if reply["seat"] != "unknown":
                order["seat"] = reply["seat"]
                return True
            if reply["intent"] == "other":
                return False
            print("ロボット：申し訳ございません。もう一度お聞かせください。")
        return False

    # ===== サーバーへの注文 =====
    def serve(self, bowl, seat):
        """サーバーに注文を1つ出し、途中経過を表示しながら、結果を待って返す。つながらなければ None。"""
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

    # ===== 1回の頼み =====
    def take_order(self, text):
        """亭主の頼みを1つ受けて、確かめてからお出しし、報告して、記録を1行書き足す。"""
        reading = understand_orders(text)
        record = {"time": datetime.now().isoformat(), "text": text, "understanding": reading,
                  "replies": [], "auto": [], "results": []}
        try:
            record["outcome"] = self.process(reading, record)
        finally:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")

    def cancel(self, outcome):
        print("ロボット：かしこまりました。このたびは控えさせていただきます。")
        return outcome

    def process(self, reading, record):
        """①断る ②茶碗・席を埋める ③決まりを確かめる ④復唱する ⑤お正客様から順にお出しする。結果の言葉を返す。"""
        # ① 頼みではなければ断る
        if reading["intent"] == "other":
            print("ロボット：申し訳ございません。お茶をお出しすること以外は、承れません。")
            return "お断り（お茶をお出しする頼みではない）"

        # ② 茶碗を埋める（お正客様の注文から順に）→ 席を埋める
        orders = sorted((dict(o) for o in reading["orders"]), key=seat_rank)
        for order in orders:
            if order["bowl"] != "unknown":
                continue
            known = [o["bowl"] for o in orders if o is not order and o["bowl"] != "unknown"]
            if len(orders) == 2 and len(known) == 1:          # もう片方が決まっていれば、残りの茶碗
                order["bowl"] = other_bowl(known[0])
                record["auto"].append({"seat": order["seat"], "bowl": order["bowl"], "reason": "残りの茶碗"})
                continue
            if not self.ask_bowl(order, record):
                return self.cancel("取りやめ（茶碗が分からなかった）")
        for order in orders:
            if order["seat"] == "unknown" and not self.ask_seat(order, record):
                return self.cancel("取りやめ（席が分からなかった）")

        # ③ 決まりに合うかを確かめる
        if len({o["bowl"] for o in orders}) < len(orders):
            print("ロボット：申し訳ございません。同じお茶碗を、2つのお席にはお出しできません。")
            return "お断り（同じ茶碗を2つの席へ）"
        if len({o["seat"] for o in orders}) < len(orders):
            print("ロボット：申し訳ございません。同じお席に、2つのお茶碗はお出しできません。")
            return "お断り（同じ席に2つの茶碗）"
        orders.sort(key=seat_rank)                            # いつもお正客様から
        record["orders"] = orders

        # ④ 復唱して確かめる
        sources = [reading["source"]] + [r["understanding"]["source"] for r in record["replies"]]
        note = "（予備の方法で読み取りました。よくお確かめください）" if "keyword" in sources else ""
        if len(orders) == 1:
            plan = f"{BOWL_JA[orders[0]['bowl']]}を、{SEAT_JA[orders[0]['seat']]}にお出しいたします。"
        else:
            plan = "、".join(f"{BOWL_JA[o['bowl']]}を{SEAT_JA[o['seat']]}に" for o in orders) + "、この順にお出しいたします。"
        answer = input(f"ロボット：{plan}よろしいですか？{note}（はい／いいえ）\n亭主：").strip()
        record["confirm"] = answer
        if not is_yes(answer):
            print("ロボット：かしこまりました。取りやめます。")
            return "取りやめ（確認で「はい」がなかった）"

        # ⑤ お正客様から順にお出しする。失敗したら、残りはお出ししない
        for i, order in enumerate(orders):
            if len(orders) > 1:
                print(f"ロボット：{i + 1}つ目、{BOWL_JA[order['bowl']]}を{SEAT_JA[order['seat']]}にお出しいたします。")
            result = self.serve(order["bowl"], order["seat"])
            record["results"].append({"order": order, "result": None if result is None else {
                "success": result.success, "message": result.message, "failed_step": result.failed_step,
                "placement_error_m": result.placement_error_m, "max_tilt_deg": result.max_tilt_deg,
                "elapsed_sec": result.elapsed_sec}})
            if result is None or not result.success:
                reason = "お出しする係に連絡がつきません。" if result is None else result.message
                print(f"ロボット：申し訳ございません。{reason}")
                if i < len(orders) - 1:
                    print("ロボット：2つ目は、お出しするのを控えます。")
                step = "サーバーにつながらない" if result is None else result.failed_step
                return f"失敗（{i + 1}つ目：{step}）"
            print(f"ロボット：{BOWL_JA[order['bowl']]}を、{SEAT_JA[order['seat']]}にお出しいたしました"
                  f"（ずれ {result.placement_error_m * 1000:.1f}mm・傾き {result.max_tilt_deg:.1f}度・"
                  f"{result.elapsed_sec:.0f}秒）")
        return "成功" if len(orders) == 1 else f"成功（{len(orders)}つ）"


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
        executor.shutdown()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()