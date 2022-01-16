/**
 * @file mental_todo_list_node.cpp
 * @brief
 * @date 2022-01-15
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <twelite_interfaces/TweliteAppCueMsg.h>

#define NODE_NAME_MENTAL_TODO_LIST "mental_todo_list_node"               /*!< mental todo listノードID */
#define GROUP_PACKAGES_HOME_MASIRO "/masiro"                             /*!< ホーム: masiro */
#define TOPIC_NAME_VOICE_ID        GROUP_PACKAGES_HOME_MASIRO "/voiceID" /*!< TOPIC : /masiro/voiceID */
#define TOPIC_NAME_TWELITE_APP_CUE "/MonoWireless/TWELITE/app_cue"       /*!< TOPIC : /MonoWireless/TWELITE/app_cue */

#define debug_printf(...) printf(__VA_ARGS__)

ros::Publisher pub_to_VoiceId;
#define DELAY_COUNTER_START 8
#define DELAY_COUNTER_END   4
int value_delay_counter = 0;

enum VOICE_ID_list_t
{
    VOICE_ID_CANCEL = 20,
    VOICE_ID_CLOSE  = 27,

};

VOICE_ID_list_t current_voiceId = VOICE_ID_CANCEL;

void send_VoiceId(VOICE_ID_list_t list)
{
    std_msgs::Int64 msg;

    msg.data = list;

    debug_printf("send_VoiceId:[%ld]\n", msg.data);
    pub_to_VoiceId.publish(msg);
}

void received_TWELITEMessage(const twelite_interfaces::TweliteAppCueMsg &msg)
{
    switch (msg.event_id.data) {
        case msg.TWELITE_EVENT_DICE_SHAKE:
        case msg.TWELITE_EVENT_DICE_MOVE:
            value_delay_counter = DELAY_COUNTER_END;
            debug_printf("received_TWELITEMessage : %ld\n", msg.event_id.data);
            break;

        default:
            //value_delay_counter = DELAY_COUNTER_END;
            debug_printf("received_TWELITEMessage NOT send: %ld\n", msg.event_id.data);
            break;
    }
}

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME_MENTAL_TODO_LIST);
    ros::NodeHandle n;

    ros::Subscriber sub_TWELITEMessage = n.subscribe(TOPIC_NAME_TWELITE_APP_CUE, 2000, received_TWELITEMessage);

    pub_to_VoiceId = n.advertise<std_msgs::Int64>(TOPIC_NAME_VOICE_ID, 10);

    ros::Rate loop_rate(10); // 10Hz
    bool received_flag = false;
    int counter        = DELAY_COUNTER_START;

    while (ros::ok()) {
        ros::spinOnce();

        if (0 < value_delay_counter) {
            if (false == received_flag) {
                counter       = DELAY_COUNTER_START;
                received_flag = true;
                debug_printf("Flag on\n");
            }
            value_delay_counter--;
        }
        if (true == received_flag) {
            if (0 > counter) {
            } else if (3 > counter) {
                send_VoiceId(VOICE_ID_CLOSE);
                value_delay_counter = DELAY_COUNTER_END;
                counter--;
            } else {
                counter--;
            }
            debug_printf("counter[%d]\n", counter);
        }
        if (0 >= value_delay_counter) {
            if (true == received_flag) {
                if (0 >= value_delay_counter) {
                    send_VoiceId(VOICE_ID_CANCEL);
                    received_flag = false;
                    debug_printf("Flag off\n");
                }
            }
        }

        loop_rate.sleep();
    }

    return 0;
}
