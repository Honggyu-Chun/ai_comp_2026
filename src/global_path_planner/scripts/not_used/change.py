#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import os

class strip_z_from_txt:
    def __init__(self):
        """
        입력: path_data/<input_filename> (기본: input_xyz.txt)
             각 줄 형식: x y z (공백/탭 구분) — z는 있어도 되고 없어도 됨(최소 x,y 필요)
        출력: path_data/<output_filename> (기본: 1_test_global_path.txt)
             각 줄 형식: x \t y  (탭 구분, 당신 코드와 동일)
        파일명 충돌 시: <이름>_1.txt, <이름>_2.txt ... 로 증가 저장 (당신 코드와 동일)
        """
        rospy.init_node('change', anonymous=True)

        # rosparam 으로 파일명 지정 가능
        input_filename  = rospy.get_param("~input_filename",  "25molit_ai_mission.txt")
        output_filename = rospy.get_param("~output_filename", "test.txt")

        # 패키지 경로/폴더 준비
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('global_path_planner')
        directory = os.path.join(pkg_path, "path_data")
        if not os.path.exists(directory):
            os.makedirs(directory)

        # 입력/출력 경로
        self.input_path = os.path.join(directory, input_filename)
        self.output_path = self._unique_output_path(os.path.join(directory, output_filename))

        # 변환 실행
        self._convert_xyz_to_xy()

        # 종료
        rospy.signal_shutdown("done")

    def _unique_output_path(self, path):
        """ 기존 파일 있으면 _1, _2 ... 붙이는 로직 (당신 코드와 동일 스타일) """
        base, ext = os.path.splitext(path)
        i = 1
        candidate = path
        while os.path.exists(candidate):
            candidate = f"{base}_{i}{ext}"
            i += 1
        return candidate

    def _convert_xyz_to_xy(self):
        if not os.path.exists(self.input_path):
            rospy.logerr("Input file not found: %s", self.input_path)
            return

        lines_in = 0
        lines_out = 0

        with open(self.input_path, "r", encoding="utf-8") as fin, \
             open(self.output_path, "w", encoding="utf-8") as fout:
            for ln, line in enumerate(fin, start=1):
                lines_in += 1
                parts = line.strip().split()
                if len(parts) < 2:
                    # 최소 x,y 없으면 스킵
                    rospy.logwarn("Skip line %d (need at least x y): %s", ln, line.strip())
                    continue
                # 앞의 두 값만 기록 (탭 구분, 개행 포함) — 당신 코드 포맷과 동일
                fout.write("{0}\t{1}\n".format(parts[0], parts[1]))
                lines_out += 1

        rospy.loginfo("Converted xyz->xy: %s -> %s (in:%d, out:%d)",
                      os.path.basename(self.input_path),
                      os.path.basename(self.output_path),
                      lines_in, lines_out)

if __name__ == '__main__':
    try:
        strip_z_from_txt()
    except rospy.ROSInterruptException:
        pass
