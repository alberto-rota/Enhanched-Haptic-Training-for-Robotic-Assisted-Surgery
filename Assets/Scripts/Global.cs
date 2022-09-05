// Copyright (c) 2022 Alberto Rota
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Global
{
    public static string subjectID = "X";

    public static float assistance = 1f;

    public static bool debugmode = false;

    public static bool vfactive = false;

    public const string tooltip_path = @"/ROBOT/world/base_link/suj_psm1_L0/suj_psm1_L1/suj_psm1_L2/suj_psm1_L3/"+
    "suj_psm1_L4/psm1/psm1_base_link/psm1_yaw_link/psm1_pitch_back_link/Collisions/psm1_pitch_end_link/"+
    "psm1_main_insertion_link/psm1_tool_roll_link/psm1_tool_pitch_link/psm1_tool_yaw_link/psm1_tool_gripper2_link/PSM_TIP";

    public const string tooltip_path2 = @"/ROBOT/world/base_link/suj_psm2_L0/suj_psm2_L1/suj_psm2_L2/suj_psm2_L3/"+
    "suj_psm2_L4/psm2/psm2_base_link/psm2_yaw_link/psm2_pitch_back_link/psm2_pitch_bottom_link/psm2_pitch_end_link/"+
    "psm2_main_insertion_link/psm2_tool_roll_link/psm2_tool_pitch_link/psm2_tool_yaw_link/psm2_tool_gripper2_link/PSM_TIP2";

    public const string tooltip_fix_path = @"/ROBOT/world/base_link/suj_psm1_L0/suj_psm1_L1/suj_psm1_L2/"+
    "suj_psm1_L3/suj_psm1_L4/psm1/psm1_base_link/psm1_yaw_link/psm1_pitch_back_link/Collisions/"+
    "psm1_pitch_end_link/psm1_main_insertion_link/PSM_TIP_FIX";

    public const string rcm_path = @"/ROBOT/world/base_link/suj_ecm_L0/suj_ecm_L1/suj_ecm_L2/suj_ecm_L3/"+
    "ecm_base_link/ecm_yaw_link/ecm_pitch_front_link/ecm_pitch_bottom_link/ecm_pitch_end_link/ecm_main_insertion_link/ecm_remote_center_link";

    public const string ecm_path = @"/ROBOT/world/base_link/suj_ecm_L0/suj_ecm_L1/suj_ecm_L2/suj_ecm_L3/ecm_base_link/ecm_yaw_link/"+
    "ecm_pitch_front_link/ecm_pitch_bottom_link/ecm_pitch_end_link/ecm_main_insertion_link/ecm_tool_link/ecm_end_link";
    
    public const string pinch2 = @"PSM/outer_yaw_joint/outer_yaw_joint_revolute/outer_pitch_joint"+
    "/outer_pitch_joint_revolute/outer_insertion_joint/outer_insertion_joint_prismatic/"+
    "outer_roll_joint/outer_roll_joint_revolute/outer_wrist_pitch_joint/"+
    "outer_wrist_pitch_joint_revolute/outer_wrist_yaw_joint/outer_wrist_yaw_joint_revolute/jaw_mimic_2_joint";

    public const string pinch1 = @"PSM/outer_yaw_joint/outer_yaw_joint_revolute/outer_pitch_joint"+
    "/outer_pitch_joint_revolute/outer_insertion_joint/outer_insertion_joint_prismatic/"+
    "outer_roll_joint/outer_roll_joint_revolute/outer_wrist_pitch_joint/"+
    "outer_wrist_pitch_joint_revolute/outer_wrist_yaw_joint/outer_wrist_yaw_joint_revolute/jaw_mimic_1_joint";

    public static void Arrow(Vector3 from, Vector3 to, Color color) {
        int coneResolution=20;
        float deltaTheta = 360f/coneResolution;

        Vector3 stem = (to-from)*0.9f;
        Vector3 tip = (to-from)-stem;
        float tipradius = 0.05f*(to-from).magnitude;
        List<Vector3> tipBasePoints = new List<Vector3>();
        Vector3 b = Vector3.Cross(tip.normalized, Vector3.up)*tipradius;
        tipBasePoints.Add(b);

        for (int i=0; i<coneResolution-1; i++) {
            float theta = deltaTheta*i; 
            b = Quaternion.AngleAxis(deltaTheta,tip.normalized)*b;
            tipBasePoints.Add(b);
        }
        Vector3 tipcenter = from+stem;
        //DRAWING THE STEM
        Debug.DrawLine(from, tipcenter, color);
        // DRAWING THE TIP
        for (int i=0; i<coneResolution; i++) {
            Debug.DrawLine(tipcenter+tipBasePoints[i],to, color);
            if (i==coneResolution-1) Debug.DrawLine(tipcenter+tipBasePoints[i],tipcenter+tipBasePoints[0],color);
            else Debug.DrawLine(tipcenter+tipBasePoints[i],tipcenter+tipBasePoints[i+1],color);
        }
    }

    public static float DistMapAttraction(float d, float threshold, float half, float slope) {
        return 1/(1+Mathf.Exp(-slope/0.2f*(d-threshold-half)));
    }

    public static float AngleMapAttraction(float a, float threshold, float half, float slope) {
        return 1/(1+Mathf.Exp(-slope/0.2f*(a-threshold-half)));
    }

    public static float DistMapRepulsion(float d, float threshold, float half, float slope) {
        return 1/(1+Mathf.Exp(slope/0.2f*(d-threshold-half)));
    }

    public static void DebugOnHRSV(string text) {
        GameObject.Find("/Text/CanvasDebug/DebugText").GetComponent<UnityEngine.UI.Text>().text += text+"\n";
        GameObject.Find("/Text/CanvasDebugL/DebugTextL").GetComponent<UnityEngine.UI.Text>().text += text+"\n";
    }
}