using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class ScoreManager : MonoBehaviour
{
    public Text redScoreText;
    public Text blueScoreText;
    public Text winnerText;

    public List<GameObject> redPucks = new List<GameObject>();
    public List<GameObject> bluePucks = new List<GameObject>();

    public PlayerController playerController;

    private int redScore = 0;
    private int blueScore = 0;
    private bool scoreUpdated = false;
    private bool winnerDisplayed = false;

    void Start()
    {
        if (winnerText != null)
            winnerText.text = "";
        if (playerController == null)
        {
            playerController = FindObjectOfType<PlayerController>();
        }
        Debug.Log($"Found {redPucks.Count} red pucks and {bluePucks.Count} blue pucks.");
    }

    void Update()
    {
        if (AllPucksStopped())
        {
            if (!scoreUpdated)
            {
                CalculateScore();
                scoreUpdated = true;
            }

            if (playerController.Finished() && !winnerDisplayed)
            {
                Debug.Log("Displaying winner");
                DisplayWinner();
                winnerDisplayed = true;
            }
        }
        else
        {
            scoreUpdated = false;
        }
    }

    bool AllPucksStopped()
    {
        foreach (var puck in redPucks)
        {
            if (puck.GetComponent<Rigidbody>().velocity.magnitude > 0.05f)
                return false;
        }

        foreach (var puck in bluePucks)
        {
            if (puck.GetComponent<Rigidbody>().velocity.magnitude > 0.05f)
                return false;
        }

        return true;
    }

    public void CalculateScore()
    {
        redScore = 0;
        blueScore = 0;

        foreach (GameObject puckObj in redPucks)
        {
            PlayerMovement puck = puckObj.GetComponent<PlayerMovement>();
            if (puck != null && puck.HasLaunched() && !puck.HasFallen())
            {
                redScore += GetScoreFromX(puck.transform.position.x);
            }
        }

        foreach (GameObject puckObj in bluePucks)
        {
            PlayerMovement puck = puckObj.GetComponent<PlayerMovement>();
            if (puck != null && puck.HasLaunched() && !puck.HasFallen())
            {
                blueScore += GetScoreFromX(puck.transform.position.x);
            }
        }

        UpdateScoreText();
    }

    private int GetScoreFromX(float x)
    {
        if (x >= -2.20f && x < -1.965f) return 2;
        else if (x >= -2.553f && x < -2.20f) return 3;
        else return 1;
    }

    private void UpdateScoreText()
    {
        if (redScoreText != null) redScoreText.text = $"Red: {redScore}";
        if (blueScoreText != null) blueScoreText.text = $"Blue: {blueScore}";
    }

    private void DisplayWinner()
    {
        if (winnerText == null)
        {
            Debug.LogWarning("Winner text UI is not assigned.");
            return;
        }

        if (redScore > blueScore)
            winnerText.text = "Red wins!";
        else if (blueScore > redScore)
            winnerText.text = "Blue wins!";
        else
            winnerText.text = "It's a tie!";
    }
}
