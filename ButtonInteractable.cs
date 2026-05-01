using UnityEngine;

public class ButtonInteractable : MonoBehaviour {
    bool pressed = false;

    public void Press() {
        if (!pressed) {
            pressed = true;
            // Change colour to show it's been pressed
            GetComponent<Renderer>().material.color = Color.green;
            Invoke("Reset", 1f);
            Debug.Log("BUTTON FIRED");
        }
    }

    void Reset() {
        pressed = false;
        GetComponent<Renderer>().material.color = Color.white;
    }
}