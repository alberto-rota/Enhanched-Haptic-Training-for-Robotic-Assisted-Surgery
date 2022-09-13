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

    List<string> scenes = new List<string>();

    void Start() {
        gameObject.GetComponent<LogData>().enabled = false;
        if (GameObject.FindWithTag("ROBOT").GetComponent<SumForcesRL>() != null) 
           GameObject.FindWithTag("ROBOT").GetComponent<SumForcesRL>().enabled = Global.vfactive;
        if (GameObject.FindWithTag("ROBOT").GetComponent<SumForces>() != null) 
           GameObject.FindWithTag("ROBOT").GetComponent<SumForces>().enabled = Global.vfactive;
        
        // List of scene name for random selection
        scenes.Add("Training1");
        scenes.Add("Training2");
        scenes.Add("Training3");
        scenes.Add("Training4");
        scenes.Add("Thymectomy");
        scenes.Add("Nephrectomy");
        scenes.Add("LiverResection");
        scenes.Add("Suturing");
    }

    void Update()
    {
        GameObject robot = GameObject.FindWithTag("ROBOT");

        // Quits when the ESC key is pressed
        if(Input.GetKey(KeyCode.Escape)){
            Application.Quit();
        }

        // Disbles Wrench message when O is pressed
        if(Input.GetKey(KeyCode.O)){
            if (robot.GetComponent<WrenchPublisherRight>() != null) 
                robot.GetComponent<WrenchPublisherRight>().safetyOverride = !robot.GetComponent<WrenchPublisherRight>().safetyOverride;
            if (robot.GetComponent<WrenchPublisherLeft>() != null) 
                robot.GetComponent<WrenchPublisherLeft>().safetyOverride = !robot.GetComponent<WrenchPublisherLeft>().safetyOverride;
            if (robot.GetComponent<WrenchPublisher>() != null) 
                robot.GetComponent<WrenchPublisher>().safetyOverride = !robot.GetComponent<WrenchPublisher>().safetyOverride;
            if (robot.GetComponent<WrenchFTPublisherRight>() != null) 
                robot.GetComponent<WrenchFTPublisherRight>().safetyOverride = !robot.GetComponent<WrenchFTPublisherRight>().safetyOverride;
            if (robot.GetComponent<WrenchFTPublisherLeft>() != null) 
                robot.GetComponent<WrenchFTPublisherLeft>().safetyOverride = !robot.GetComponent<WrenchFTPublisherLeft>().safetyOverride;
            if (robot.GetComponent<WrenchFTPublisher>() != null) 
                robot.GetComponent<WrenchFTPublisher>().safetyOverride = !robot.GetComponent<WrenchFTPublisher>().safetyOverride;
        }
        
        // Toggles the VFs when the V key is pressed or COAG pedal is pressed
        if(Input.GetKeyDown(KeyCode.V) || gameObject.GetComponent<PedalCoagSubscriber>().pressed){
            Global.vfactive = !Global.vfactive;
        }

        if (robot.GetComponent<SumForces>() != null) {
            // Toogles the boolan for the virtual fixtures
            robot.GetComponent<SumForces>().enabled = Global.vfactive;

            if (robot.GetComponent<SumForces>().enabled) {
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF ACTIVE";
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF ACTIVE";    
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            } else {
                GameObject.FindWithTag("ROBOT").GetComponent<SumForces>().totalForce = Vector3.zero;            
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF INACTIVE";
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF INACTIVE";
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.red;
            }
        }

        if (robot.GetComponent<SumForcesRL>() != null) {

            robot.GetComponent<SumForcesRL>().enabled = Global.vfactive;

            if (robot.GetComponent<SumForcesRL>().enabled) {
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF ACTIVE";
                GameObject.Find("Text/CanvasVF/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().text="VF ACTIVE";    
                GameObject.Find("Text/CanvasVFL/VFActiveText").GetComponent<UnityEngine.UI.Text>().color=Color.green;
            } else {
                GameObject.FindWithTag("ROBOT").GetComponent<SumForcesRL>().totalForceRight = Vector3.zero;            
                GameObject.FindWithTag("ROBOT").GetComponent<SumForcesRL>().totalForceLeft = Vector3.zero;            
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
            SceneManager.LoadScene("Assets/Training4.unity");
        }else if(Input.GetKey(KeyCode.Alpha5)){
            SceneManager.LoadScene("Assets/Thymectomy.unity");
        }else if(Input.GetKey(KeyCode.Alpha6)){
            SceneManager.LoadScene("Assets/Nephrectomy.unity");
        }else if(Input.GetKey(KeyCode.Alpha7)){
            SceneManager.LoadScene("Assets/LiverResection.unity");
        }else if(Input.GetKey(KeyCode.Alpha8)){
            SceneManager.LoadScene("Assets/Suturing.unity");
        }else if(Input.GetKey(KeyCode.Alpha9)){
            SceneManager.LoadScene("Assets/Training1_LR.unity");
        }

        // The PLUS pedal goes to the next scene
        // if (gameObject.GetComponent<PedalPlusSubscriber>().pressed == true) {
        //     if(SceneManager.GetActiveScene().name == "Playground"){
        //         SceneManager.LoadScene("Assets/Training1.unity",LoadSceneMode.Single);                
        //     }else if(SceneManager.GetActiveScene().name == "Training1"){
        //         SceneManager.LoadScene("Assets/Training2.unity",LoadSceneMode.Single);                 
        //     }else if(SceneManager.GetActiveScene().name == "Training2"){
        //         SceneManager.LoadScene("Assets/Training3.unity",LoadSceneMode.Single);                
        //     }else if(SceneManager.GetActiveScene().name == "Training3"){
        //         SceneManager.LoadScene("Assets/Training4.unity",LoadSceneMode.Single);                
        //     }else if(SceneManager.GetActiveScene().name == "Training4"){
        //         SceneManager.LoadScene("Assets/Thymectomy.unity",LoadSceneMode.Single);                
        //     }else if(SceneManager.GetActiveScene().name == "Thymectomy"){
        //         SceneManager.LoadScene("Assets/Nephrectomy.unity",LoadSceneMode.Single);                
        //     }else if(SceneManager.GetActiveScene().name == "Nephrectomy"){
        //         SceneManager.LoadScene("Assets/LiverResection.unity",LoadSceneMode.Single);                
        //     }else if(SceneManager.GetActiveScene().name == "LiverResection"){
        //         SceneManager.LoadScene("Assets/Suturing.unity",LoadSceneMode.Single);                
        //     }else if(SceneManager.GetActiveScene().name == "Suturing"){
        //         SceneManager.LoadScene("Assets/Training1_LR.unity",LoadSceneMode.Single);                
        //     }else if(SceneManager.GetActiveScene().name == "Training1_LR"){
        //         SceneManager.LoadScene("Assets/Playground.unity",LoadSceneMode.Single);                
        //     }
        // }
        // The PLUS pedal goes to the next scene picked at RANDOM
        if (gameObject.GetComponent<PedalPlusSubscriber>().pressed == true) {
            SceneManager.LoadScene("Assets/"+scenes[Random.Range(0,scenes.Count)]+".unity",LoadSceneMode.Single); 
        }

        // The MINUS pedal reloads the current scene
        if (gameObject.GetComponent<PedalMinusSubscriber>().pressed == true) 
            SceneManager.LoadSceneAsync(SceneManager.GetActiveScene().name);

        // Pressind D toggles debug mode
        if (Input.GetKeyDown(KeyCode.D)) {
            Global.debugmode = !Global.debugmode;   
        }

        // Pressing the Left Mouse Button toggles the pointer visibility
        if (Input.GetMouseButton(0)) {
            GameObject.Find("/Text/CanvasPointer/PointerR").GetComponent<UnityEngine.UI.Image>().enabled = true;
            GameObject.Find("/Text/CanvasPointerL/PointerL").GetComponent<UnityEngine.UI.Image>().enabled = true;
        }else{
            GameObject.Find("/Text/CanvasPointer/PointerR").GetComponent<UnityEngine.UI.Image>().enabled = false;
            GameObject.Find("/Text/CanvasPointerL/PointerL").GetComponent<UnityEngine.UI.Image>().enabled = false;
        }
    }
}
}
