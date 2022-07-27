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

namespace RosSharp.RosBridgeClient{

public class Manager : MonoBehaviour
{
    void Start() {
        gameObject.GetComponent<LogData>().enabled = false;
        GameObject.Find("ROBOT").GetComponent<SumForces>().enabled = false;
    }

    void Update()
    {
        GameObject robot = GameObject.Find("ROBOT");

        // Quits when the ESC key is pressed
        if(Input.GetKey(KeyCode.Escape)){
            Application.Quit();
        }

        // Disbles Wrench message when O is pressed
        if(Input.GetKey(KeyCode.O)){
            robot.GetComponent<WrenchPublisher>().forceOverride = !robot.GetComponent<WrenchPublisher>().forceOverride;
        }
        
        // Toggles the VFs when the V key is pressed
        if(Input.GetKeyDown(KeyCode.V) || gameObject.GetComponent<PedalPlusSubscriber>().pressed){
            robot.GetComponent<SumForces>().enabled = !robot.GetComponent<SumForces>().enabled;
            if (robot.GetComponent<SumForces>().enabled) {
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF ACTIVE";
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF ACTIVE";
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            } else {
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF INACTIVE";
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF INACTIVE";
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
            }
        }

        // Starts data logging when the R key is pressed
        if(Input.GetKeyDown(KeyCode.R) || gameObject.GetComponent<PedalBiCoagSubscriber>().pressed){    
            if (gameObject.GetComponent<LogData>() != null){
                gameObject.GetComponent<LogData>().enabled = !gameObject.GetComponent<LogData>().enabled;
                // if (gameObject.GetComponent<LogData>().enabled) gameObject.GetComponent<LogData>().Start();
            // }else if (gameObject.GetComponent<LogDataTraining2>() != null){
            //     gameObject.GetComponent<LogDataTraining2>().enabled = !gameObject.GetComponent<LogDataTraining2>().enabled;
            //     if (gameObject.GetComponent<LogDataTraining2>().enabled) gameObject.GetComponent<LogDataTraining2>().Start();
            // }else if (gameObject.GetComponent<LogDataTraining3>() != null){
            //     gameObject.GetComponent<LogDataTraining3>().enabled = !gameObject.GetComponent<LogDataTraining3>().enabled;
            //     if (gameObject.GetComponent<LogDataTraining3>().enabled) gameObject.GetComponent<LogDataTraining3>().Start();
            // }else if (gameObject.GetComponent<LogDataLiverResection>() != null){
            //     gameObject.GetComponent<LogDataLiverResection>().enabled = !gameObject.GetComponent<LogDataLiverResection>().enabled;
            //     if (gameObject.GetComponent<LogDataLiverResection>().enabled) gameObject.GetComponent<LogDataLiverResection>().Start();
            // }else if (gameObject.GetComponent<LogDataThymectomy>() != null){
            //     gameObject.GetComponent<LogDataThymectomy>().enabled = !gameObject.GetComponent<LogDataThymectomy>().enabled;
            //     if (gameObject.GetComponent<LogDataThymectomy>().enabled) gameObject.GetComponent<LogDataThymectomy>().Start();
            // }else if (gameObject.GetComponent<LogDataNephrectomy>() != null){
            //     gameObject.GetComponent<LogDataNephrectomy>().enabled = !gameObject.GetComponent<LogDataNephrectomy>().enabled;
            //     if (gameObject.GetComponent<LogDataNephrectomy>().enabled) gameObject.GetComponent<LogDataNephrectomy>().Start();
            }

            if (gameObject.GetComponent<LogData>().enabled) {
                GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nGO";
                GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
                GameObject.Find("Text/CanvasVFL/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nGO";
                GameObject.Find("Text/CanvasVFL/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            } else {
                GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nSTOP";
                GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
                GameObject.Find("Text/CanvasVFL/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nSTOP";
                GameObject.Find("Text/CanvasVFL/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
            }
            // if (gameObject.GetComponent<LogDataTraining2>().enabled) {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nGO";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            // } else {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nSTOP";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
            // }
            // if (gameObject.GetComponent<LogDataTraining3>().enabled) {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nGO";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            // } else {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nSTOP";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
            // }
            // if (gameObject.GetComponent<LogDataThymectomy>().enabled) {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nGO";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            // } else {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nSTOP";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
            // }
            // if (gameObject.GetComponent<LogDataNephrectomy>().enabled) {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nGO";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            // } else {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nSTOP";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
            // }
            // if (gameObject.GetComponent<LogDataLiverResection>().enabled) {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nGO";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            // } else {
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().text="\n\nSTOP";
            //     GameObject.Find("Text/CanvasVF/LoggingText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
            // }
        }

        // Loads a scene of choice when the user presses the corresponding key
        if(Input.GetKey(KeyCode.Alpha0)){
            SceneManager.LoadScene("Assets/Playground.unity");
        }else if(Input.GetKey(KeyCode.Alpha1)){
            SceneManager.LoadScene("Assets/Training1.unity");
        }else if(Input.GetKey(KeyCode.Alpha2)){
            SceneManager.LoadScene("Assets/Training2.unity"); 
        }else if(Input.GetKey(KeyCode.Alpha3)){
            SceneManager.LoadScene("Assets/Training3.unity");
        }else if(Input.GetKey(KeyCode.Alpha4)){
            SceneManager.LoadScene("Assets/Thymectomy.unity");
        }else if(Input.GetKey(KeyCode.Alpha5)){
            SceneManager.LoadScene("Assets/Nephrectomy.unity");
        }else if(Input.GetKey(KeyCode.Alpha6)){
            SceneManager.LoadScene("Assets/LiverResection.unity");
        }

        if (gameObject.GetComponent<PedalCoagSubscriber>().pressed == true) {

            if(SceneManager.GetActiveScene().name == "Playground"){
                SceneManager.LoadScene("Assets/Training1.unity");
            }else if(SceneManager.GetActiveScene().name == "Training1"){
                SceneManager.LoadScene("Assets/Training2.unity"); 
            }else if(SceneManager.GetActiveScene().name == "Training2"){
                SceneManager.LoadScene("Assets/Training3.unity");
            }else if(SceneManager.GetActiveScene().name == "Training3"){
                SceneManager.LoadScene("Assets/Thymectomy.unity");
            }else if(SceneManager.GetActiveScene().name == "Thymectomy"){
                SceneManager.LoadScene("Assets/Nephrectomy.unity");
            }else if(SceneManager.GetActiveScene().name == "Nephrectomy"){
                SceneManager.LoadScene("Assets/LiverResection.unity");
            }else if(SceneManager.GetActiveScene().name == "LiverResection"){
                SceneManager.LoadScene("Assets/Playground.unity");
            }
        }
    }
}
}
