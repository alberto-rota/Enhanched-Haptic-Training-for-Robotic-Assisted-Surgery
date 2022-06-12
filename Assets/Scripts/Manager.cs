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
using UnityEngine.SceneManagement;

public class Manager : MonoBehaviour
{
   void Update()
    {
        // Quits when the ESC key is pressed
        if(Input.GetKey(KeyCode.Escape)){
            Application.Quit();
        }

        // Toggles the VFs when the V key is pressed
        if(Input.GetKeyDown(KeyCode.V)){
            GameObject.Find("PSM").GetComponent<SumForces>().enabled = !GameObject.Find("PSM").GetComponent<SumForces>().enabled;
                if (GameObject.Find("PSM").GetComponent<SumForces>().enabled) {
                    GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF ACTIVE";
                } else {
                    GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF INACTIVE";
                }
        }

        // Loads a scene of choice when the user presses the corresponding key
        if(Input.GetKey(KeyCode.Alpha1)){
            SceneManager.LoadScene("Assets/Tasks/Training1.unity");
        }else if(Input.GetKey(KeyCode.Alpha2)){
            SceneManager.LoadScene("Assets/Tasks/Training2.unity");
        }else if(Input.GetKey(KeyCode.Alpha3)){
            SceneManager.LoadScene("Assets/Tasks/Training3.unity");
        }else if(Input.GetKey(KeyCode.Alpha4)){
            SceneManager.LoadScene("Assets/Tasks/Thymectomy.unity");
        }else if(Input.GetKey(KeyCode.Alpha5)){
            SceneManager.LoadScene("Assets/Tasks/Nephrectomy.unity");
        }else if(Input.GetKey(KeyCode.Alpha6)){
            SceneManager.LoadScene("Assets/Tasks/LiverResection.unity");
        }
    }
}
