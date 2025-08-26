using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;
public class Target : MonoBehaviour
{
    private string handTag = "Hand";
    private string dotTag = "Dot";
    public float checkRadius = 0.05f;
    [SerializeField] public float scoreTime = 0f;
    private int score = 0;
    private bool isTouching = false;
    private bool prevGameEnded = false;
    Vector3 initialPosition;
    [SerializeField] Text scoreText;

    public Timer timer;
    private void Start()
    {
        scoreText.text = $"Score: {0}";
        initialPosition = transform.position;
    }
    public void Hit()
    {
        transform.position = Box.Instance.GetRandomPosition();
        isTouching = false;
    }

    void Update()
    {
        bool currentlyTouching = false;
        if (timer.IsGameStarted())
        {
            Collider[] hits = Physics.OverlapSphere(transform.position, checkRadius);
            foreach (var hit in hits)
            {
                if (hit.CompareTag(dotTag) || hit.CompareTag(handTag))
                {
                    currentlyTouching = true;
                    break;
                }
            }
            if (timer.isGameStarted)
            {
                scoreTime += Time.deltaTime;
            }

            if (currentlyTouching && !isTouching)
            {
                Debug.Log(scoreTime);
                isTouching = true;
                //Debug.Log($"{name} TOUCH START with {handTag}");
                //Debug.Log($"{name} TOUCH START with {dotTag}");
                Hit();
                score++;
                scoreText.text = $"Score: {score}";
            }
            else if (!currentlyTouching && isTouching)
            {
                isTouching = false;
                //Debug.Log($"{name} TOUCH END with {handTag}");
                //Debug.Log($"{name} TOUCH END with {dotTag}");
            }
        }
        else if (Timer.GameEnded) 
        {
            scoreTime = 0f;
            score = 0;
            scoreText.text = $"Score: {0}";
        }
        if (Timer.GameEnded && !prevGameEnded)
        {
            transform.position = initialPosition;
        }
        prevGameEnded = Timer.GameEnded;
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, checkRadius);
    }
}
