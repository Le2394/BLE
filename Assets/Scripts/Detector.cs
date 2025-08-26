using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Detector : MonoBehaviour
{
    [SerializeField] private Spawner spawner;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerEnter(Collider other)
    {
        bool isCorrect = spawner.CheckGoal(other.gameObject);

        if (isCorrect)
        {
            spawner.Continue();
        }

        Destroy(other.gameObject);
    }
}
