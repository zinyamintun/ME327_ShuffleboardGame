using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    public List<GameObject> puckObjects;
    public CameraFollow cameraFollow;

    private int currentTurn = 0;
    private bool finished = false;

    void Start()
    {
        foreach (GameObject puck in puckObjects)
        {
            puck.SetActive(false);
            puck.GetComponent<PlayerMovement>().isActivePuck = false;
        }

        StartTurn();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            EndTurn();
        }
    }

    void StartTurn()
    {
        if (currentTurn >= puckObjects.Count)
        {
            Debug.Log("All turns done.");
            finished = true;
            return;
        }

        GameObject currentPuck = puckObjects[currentTurn];
        currentPuck.SetActive(true);
        currentPuck.GetComponent<PlayerMovement>().isActivePuck = true;

        cameraFollow.target = currentPuck.transform;
    }

    void EndTurn()
    {
        currentTurn++;
        StartTurn();
    }

    public bool Finished()
    {
        return finished;
    }
}
