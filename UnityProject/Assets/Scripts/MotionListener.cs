using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotionListener : MonoBehaviour
{
    public const int MAX_JOYSTICK_VALUE = 512;

    [SerializeField] private SerialController serialController;

    [SerializeField] private Transform target;

    [Header("Translation")]
    public Space translation_space = Space.World;
    [SerializeField] private Vector3 translation_scale = new Vector3(1, 1, 0.1f);

    [Header("Joystick")]
    [SerializeField] private Vector2 joystick_deadzone = new Vector2(-30, 30);
    [SerializeField, HideInInspector] private Vector2 joystick_max = new Vector2(-MAX_JOYSTICK_VALUE + 30, MAX_JOYSTICK_VALUE - 30);

    [Header("Ultrasonic Sensor")]
    [SerializeField] private Vector2 ultrasonic_deadzone = new Vector2(40, 60);
    [SerializeField] private Vector2 ultrasonic_extents = new Vector2(1, 99);


    private void Reset()
    {
        serialController = FindObjectOfType<SerialController>();
        target = transform;
    }

    private void OnValidate()
    {
        if (joystick_deadzone.x < -256)
            joystick_deadzone.x = -256;
        else if (joystick_deadzone.x >= 0)
            joystick_deadzone.x = -1;

        if (joystick_deadzone.y > 256)
            joystick_deadzone.y = 256;
        else if (joystick_deadzone.y <= 0)
            joystick_deadzone.y = 1;

        joystick_max.x = -MAX_JOYSTICK_VALUE + joystick_deadzone.x;
        joystick_max.y = MAX_JOYSTICK_VALUE - joystick_deadzone.y;

        if (ultrasonic_deadzone.x <= 1)
            ultrasonic_deadzone.x = 2;

        if (ultrasonic_deadzone.y < ultrasonic_deadzone.x)
            ultrasonic_deadzone.y = ultrasonic_deadzone.x;

        if (ultrasonic_extents.x >= ultrasonic_deadzone.x)
            ultrasonic_extents.x = ultrasonic_deadzone.x - 1;

        if (ultrasonic_extents.y <= ultrasonic_deadzone.y)
            ultrasonic_extents.y = ultrasonic_deadzone.y + 1;
    }

    private void Start()
    {
        if (target == null)
            target = transform;
    }

    private void Update()
    {
        string message = serialController.ReadSerialMessage();

        if (message == null)
            return;

        string[] input = message.Split(',');

        if (input.Length < 7)
            return;

        int index = 0;
        HandleQuaternion(input, ref index);
        HandleTranslation(input, ref index);
    }

    public void HandleQuaternion(string[] motion, ref int start_index)
    {
        Quaternion q = Quaternion.identity;

        if (float.TryParse(motion[start_index], out float result))
            q.w = -result;
        
        start_index++;
        if (float.TryParse(motion[start_index], out result))
            q.x = result;

        start_index++;
        if (float.TryParse(motion[start_index], out result))
            q.z = result;
        
        start_index++;
        if (float.TryParse(motion[start_index], out result))
            q.y = result;
        
        start_index++;
        target.rotation = q;
    }

    public void HandleTranslation(string[] motion, ref int start_index)
    {
        Vector3 t = Vector3.zero;

        if (float.TryParse(motion[start_index], out float result))
            //x axis from joystick -> z axis in world
            t.z = translation_scale.x * GetNormalizedJoystickAxis(result);
        
        start_index++;
        if (float.TryParse(motion[start_index], out result))
            //y axis from joystick -> x axis in world
            t.x = translation_scale.z * GetNormalizedJoystickAxis(result);
        
        start_index++;
        if (float.TryParse(motion[start_index], out result))
        {
            if(result == 0)
                t.y = 0;
            else
            {
                if(result <= ultrasonic_deadzone.x)
                {
                    t.y = -translation_scale.y * Mathf.InverseLerp(ultrasonic_deadzone.x, ultrasonic_extents.x, result);

                }
                else if(result >= ultrasonic_deadzone.y)
                {
                    if (result > ultrasonic_extents.y)
                        result = ultrasonic_extents.y;

                    t.y = translation_scale.y * Mathf.InverseLerp(ultrasonic_deadzone.y, ultrasonic_extents.y, result);
                }
            }
        }
        
        start_index++;

        if (translation_space == Space.Self)
            t = target.rotation * t;
        
        target.position += t * Time.deltaTime;
    }

    private float GetNormalizedJoystickAxis(float rawInput)
    {
        if (rawInput <= joystick_deadzone.x)
            return -Mathf.InverseLerp(joystick_deadzone.x, joystick_max.x, rawInput);
        else if (rawInput >= joystick_deadzone.y)
            return Mathf.InverseLerp(joystick_deadzone.y, joystick_max.y, rawInput);
        else
            return 0;
    }
}
