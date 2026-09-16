# 高度制御アルゴリズムモジュール

class AltitudeController:
    def __init__(self, p_gain=5.0, max_pitch=45.0):
        """
        p_gain: 目標との差(m)に対して、どれくらいPitch(deg)を傾けるかの係数
        max_pitch: 機体がひっくり返らないための安全リミット(deg)
        """
        self.target_alt = 0.0
        self.p_gain = p_gain
        self.max_pitch = max_pitch

    def set_target(self, target_alt):
        self.target_alt = target_alt

    def get_target(self):
        return self.target_alt

    def calc_pitch_command(self, current_alt):
        """現在の高度を受け取り、送信すべきPitch指令値を計算して返す"""
        # エラー（目標までの距離）を計算
        error = self.target_alt - current_alt
        
        # P制御によるPitch量の計算（目標より低ければプラス(機首上げ)、高ければマイナス）
        pitch_cmd = error * self.p_gain
        
        # 安全のためのリミット処理
        pitch_cmd = max(min(pitch_cmd, self.max_pitch), -self.max_pitch)
        
        return pitch_cmd