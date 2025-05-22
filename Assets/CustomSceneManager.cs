//Custom Scene Manager
//Brendan Bovenschen
//Handles the changing of scene in the user interface

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class CustomSceneManager : MonoBehaviour
{
    public void ToScene(string sceneStr)
    {
        SceneManager.LoadScene(sceneStr);
    }

    public void Quit()
    {
        Application.Quit();
    }
}
