using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class Spawner : MonoBehaviour
{

    [Header("Prefabs And Materials")]
    [SerializeField] private GameObject[] shapeList;
    [SerializeField] private Material[] materialList;

    static public Transform[] spawnPoints;

    static public Dictionary<GameObject, string> shapeDict;
    static public Dictionary<Material, string> matDict;


    //Đồ vật mục tiêu
    private GameObject goal;
    public Transform goalContainer;
    public Transform goalSpawnPoint;

    private SpawnedShape currentSpawnedShape = new SpawnedShape();
    public Transform currentContainer;

    private SpawnedShape nextSpawnedShape =  new SpawnedShape();
    public Transform nextContainer;

    public Text Score;
    private int score = 0;

    void Start()
    {
        currentSpawnedShape.GenerateObject(currentContainer); 
        nextSpawnedShape.GenerateObject(nextContainer, false);

        GenerateGoal();
        Score.text = "Score: " + score.ToString();
    }

    void LateUpdate()
    {
        
    }

    void Awake()
    {
        GameObject parent = GameObject.Find("Spawn Points");

        spawnPoints = new Transform[3]
        {
            parent.transform.Find("_1"),
            parent.transform.Find("_2"),
            parent.transform.Find("_3"),
        };

        shapeDict = new Dictionary<GameObject, string>()
        {
            {shapeList[0], "hình hộp"},
            {shapeList[1], "hình trụ"},
            {shapeList[2], "hình con nhộng"},
            {shapeList[3], "hình cầu"}
        };

        matDict = new Dictionary<Material, string>()
        {
            {materialList[0], "màu đỏ"},
            {materialList[1], "màu vàng"},
            {materialList[2], "màu xanh"},
            {materialList[3], "màu tím"}
        };
    }


    public void Continue()
    {
        Destroy(goal);

        currentSpawnedShape.DestroyInstance();
        nextSpawnedShape.SetActive(true);
        currentSpawnedShape = nextSpawnedShape;

        GenerateGoal();
        score++;
        Score.text = "Score: " + score.ToString();
        nextSpawnedShape = new SpawnedShape();
        nextSpawnedShape.GenerateObject(nextContainer, false);
       
    }

    public void Restart()
    {
        Destroy(goal);

        currentSpawnedShape.DestroyInstance();
        nextSpawnedShape.SetActive(true);
        currentSpawnedShape = nextSpawnedShape;

        GenerateGoal();
        score = 0;
        Score.text = "Score: " + score.ToString();
        nextSpawnedShape = new SpawnedShape();
        nextSpawnedShape.GenerateObject(nextContainer, false);

    }

    public bool CheckGoal(GameObject triedObject)
    {
        string triedObjectName = triedObject.name.Replace("(Clone)", "").Trim();
        string goalName = goal.name.Replace("(Clone)", "").Trim();
        bool isCorrectShape = triedObjectName.Equals(goalName);

        var triedMat = triedObject.GetComponent<Renderer>().material;
        var goalMat = goal.GetComponent<Renderer>().material;

        bool isCorrectMaterial = triedMat.name.Equals(goalMat.name);

        return isCorrectShape && isCorrectMaterial;
    }



    private void GenerateGoal()
    {
        int index = UnityEngine.Random.Range(0, currentSpawnedShape.Count);
        goal = Instantiate(
                    currentSpawnedShape.GetObject(index),
                    goalSpawnPoint.position,
                    goalSpawnPoint.rotation,
                    goalContainer
                    );


        goal.transform.localScale = new Vector3(0.08f, 0.08f, 0.08f);
        //Destroy(goal.GetComponent<MoveAround>());

    }

    private class SpawnedShape
    {
        public int Count;
        private GameObject[] objectsList;

        public SpawnedShape(int count = 3)
        {
            Count = count;
            objectsList = new GameObject[count];
        }

        public void GenerateObject(Transform parent, Boolean active = true)
        {
            for(int i = 0; i < objectsList.Length; i++)
            {
                KeyValuePair<GameObject, string> objectPair = GetRandomEntry(shapeDict);
                KeyValuePair<Material, string> materialPair = GetRandomEntry(matDict);

                objectsList[i] = Instantiate(
                    objectPair.Key,
                    spawnPoints[i].position,
                    spawnPoints[i].rotation,
                    parent
                    );

                if (objectsList[i].GetComponent<Rigidbody>() == null)
                {
                    objectsList[i].AddComponent<Rigidbody>();
                }

                objectsList[i].GetComponent<Renderer>().material = materialPair.Key;

                if (objectsList[i].GetComponent<DragHandler>() == null)
                {
                    objectsList[i].AddComponent<DragHandler>();
                }

                objectsList[i].SetActive(active);
            }
        }

        public void DestroyInstance()
        {
          
            for (int i = 0; i < objectsList.Length; i++)
            {
                if (objectsList[i] != null)
                    Destroy(objectsList[i]);
            }
           
        }

        public void SetActive(Boolean active)
        {
            try
            {
                for (int i = 0; i < objectsList.Length; i++)
                {
                    objectsList[i].SetActive(active);
                }
            } catch(NullReferenceException e)
            {
                Debug.Log("Error" + e.Message);
            }
           
        }

        private KeyValuePair<T1, T2> GetRandomEntry<T1, T2>(Dictionary<T1, T2> dict)
        {
            int index = UnityEngine.Random.Range(0, dict.Count);
            int i = 0;

            foreach (var pair in dict)
            {
                if (i == index)
                    return pair;
                i++;
            }

            return default;
        }

        public GameObject GetObject(int index)
        {
            return objectsList[index];
        }

    }
}
