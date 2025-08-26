using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class Timer : MonoBehaviour
{
    public static Action OnGameEnded;
    public ESP32BleReceiver bleReceiver;
    public static bool GameEnded { get; private set; }
    private bool countdownStarted = false;

    [SerializeField] Text timerText;
    [SerializeField] Text countdownText;

    float endTime;
    public float gameTime = 10f;
    public int countdownTime = 3;
    public bool isGameStarted = false;
    void Start()
    {
        GameEnded = false;
        timerText.text = $"Time: {gameTime}";
    }
    public bool IsGameStarted()
    {
        return isGameStarted;
    }
    private void Update()
    {
        if (!countdownStarted && bleReceiver.IsPalmPrinting())
        {
            countdownStarted = true;
            StartCoroutine(StartCountdown());
        }
    }
    IEnumerator StartCountdown()
    {
        int cnt = countdownTime;

        while (cnt > 0)
        {
            countdownText.text = cnt.ToString();
            yield return new WaitForSeconds(1);
            cnt--;
        }

        countdownText.text = "GO!";
        yield return new WaitForSeconds(1);
        GameEnded = false;
        countdownText.gameObject.SetActive(false);
        endTime = Time.time + gameTime;
        StartCoroutine(GameTimer());
    }

    IEnumerator GameTimer()
    {
        while (!GameEnded)
        {
            float timeLeft = endTime - Time.time;
            isGameStarted = true;
            if (timeLeft <= 0)
            {
                GameEnded = true;
                OnGameEnded?.Invoke();
                timeLeft = 0;
            }
            timerText.text = $"Time: {timeLeft:0.0}";
            yield return null;
        }
        isGameStarted = false;
        countdownText.gameObject.SetActive(true);
        countdownText.text = "RELAX";
        timerText.text = $"Time: {gameTime}";
        yield return new WaitForSeconds(3);
        StartCoroutine(StartCountdown());
    }
}
