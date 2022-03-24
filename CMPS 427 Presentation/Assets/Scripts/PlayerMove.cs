using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerMove : MonoBehaviour
{
    [SerializeField]
    private Rigidbody playerBody;
    Vector3 jumpForce = new Vector3(0, 10, 0);

    private Vector3 inputVector;


    private void Start()
    {
        playerBody = GetComponent<Rigidbody>();

    }

    private void Update()
    {
        inputVector = new Vector3(Input.GetAxis("Horizontal") * 10f, playerBody.velocity.y, Input.GetAxis("Vertical") * 10f);
        transform.LookAt(transform.position + new Vector3(inputVector.x, 0, inputVector.z));

        
        if (Input.GetKeyDown(KeyCode.Space))
        {
            playerBody.AddForce(jumpForce, ForceMode.Impulse);
        }
    }

    private void FixedUpdate()
    {
        playerBody.velocity = inputVector;
    }
}
