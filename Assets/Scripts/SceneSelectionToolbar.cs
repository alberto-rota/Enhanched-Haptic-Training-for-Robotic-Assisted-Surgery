
// CREDITS: Diego Giacomelli
// All copyrights reserved to this script are entitled to Diego Giacomelli.

// Scene Selection Toolbar - http://diegogiacomelli.com.br/unitytips-scene-selection-toolbar
#if (UNITY_EDITOR)
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityToolbarExtender;

[InitializeOnLoad]
public static class SceneSelectionToolbar
{
    static List<SceneInfo> _scenes;
    static SceneInfo _sceneOpened;    
    static int _selectedIndex;
    static string[] _displayedOptions;

    static SceneSelectionToolbar()
    {
        LoadFromPlayerPrefs();
        ToolbarExtender.LeftToolbarGUI.Add(OnToolbarGUI);
        EditorSceneManager.sceneOpened += HandleSceneOpened;            
    } 

    static void OnToolbarGUI()
    {
        GUILayout.FlexibleSpace();
        
        _selectedIndex = EditorGUILayout.Popup(_selectedIndex, _displayedOptions); ;

        GUI.enabled = _selectedIndex == 0;
        if (GUILayout.Button("+"))
            AddScene(_sceneOpened);

        GUI.enabled = _selectedIndex > 0;
        if (GUILayout.Button("-"))
            RemoveScene(_sceneOpened);

        GUI.enabled = true;
        if (GUI.changed && _selectedIndex > 0 && _scenes.Count > _selectedIndex - 1)
            EditorSceneManager.OpenScene(_scenes[_selectedIndex - 1].Path);
    }

    static void RefreshDisplayedOptions()
    {
        _displayedOptions = new string[_scenes.Count + 1];
        _displayedOptions[0] = "Click on '+' to add current scene";

        for (int i = 0; i < _scenes.Count; i++)
            _displayedOptions[i + 1] = _scenes[i].Name;
    }

    static void HandleSceneOpened(Scene scene, OpenSceneMode mode) => SetOpenedScene(scene);

    static void SetOpenedScene(SceneInfo scene)
    {
        if (scene == null || string.IsNullOrEmpty(scene.Path))
            return;

        for (int i = 0; i < _scenes.Count; i++)
        {
            if (_scenes[i].Path == scene.Path)
            {
                _sceneOpened = _scenes[i];
                _selectedIndex = i + 1;
                SaveToPlayerPrefs(true);
                return;
            }
        }

        _sceneOpened = scene;
        _selectedIndex = 0;
        SaveToPlayerPrefs(true);
    }

    static void SetOpenedScene(Scene scene) => SetOpenedScene(new SceneInfo(scene));

    static void AddScene(SceneInfo scene)
    {
        if (scene == null)
            return;

        if (_scenes.Any(s => s.Path == scene.Path))
            RemoveScene(scene);

        _scenes.Add(scene);
        _selectedIndex = _scenes.Count;
        SetOpenedScene(scene);
        RefreshDisplayedOptions();
        SaveToPlayerPrefs();
    }

    static void RemoveScene(SceneInfo scene)
    {
        _scenes.Remove(scene);
        _selectedIndex = 0;                
        RefreshDisplayedOptions();
        SaveToPlayerPrefs(); 
    }

    static void SaveToPlayerPrefs(bool onlyLatestOpenedScene = false)
    {
        if (!onlyLatestOpenedScene)
        {
            var serialized = string.Join(";", _scenes.Where(s => !string.IsNullOrEmpty(s.Path)).Select(s => s.Path));
            SetPref("SceneSelectionToolbar.Scenes", serialized);
        }

        if(_sceneOpened != null)
            SetPref("SceneSelectionToolbar.LatestOpenedScene", _sceneOpened.Path);
    }

    static void LoadFromPlayerPrefs()
    {
        var serialized = GetPref("SceneSelectionToolbar.Scenes");
        
        _scenes = serialized.Split(new char[] { ';' }, StringSplitOptions.RemoveEmptyEntries).Select(s => new SceneInfo(s)).ToList();

        if (_scenes == null)
            _scenes = new List<SceneInfo>();        

        serialized = GetPref("SceneSelectionToolbar.LatestOpenedScene");

        if (!string.IsNullOrEmpty(serialized))
            SetOpenedScene(new SceneInfo(serialized));
        
        RefreshDisplayedOptions();
    }

    static void SetPref(string name, string value) => EditorPrefs.SetString($"{Application.productName}_{name}", value);
    static string GetPref(string name) => EditorPrefs.GetString($"{Application.productName}_{name}");

    [Serializable]
    class SceneInfo
    {
        public SceneInfo() { }
        public SceneInfo(Scene scene)
        {
            Name = scene.name;
            Path = scene.path;
        }

        public SceneInfo(string path)
        {
            Name = System.IO.Path.GetFileNameWithoutExtension(path);
            Path = path;
        }

        public string Name;
        public string Path;
    }
}
//         public SceneInfo(string path)
//         {
//             Name = System.IO.Path.GetFileNameWithoutExtension(path);
//             Path = path;
//         }

//         public string Name;
//         public string Path;
//     }
// }
#endif