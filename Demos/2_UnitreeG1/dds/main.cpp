#include <chrono>
#include <iostream>
#include <thread>

extern "C" {
#include "dds/dds.h"
#include "Twist.h"
}

int main() {
  dds_entity_t participant =
      dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
  if (participant < 0) {
    std::cerr << "Failed to create participant: "
              << dds_strretcode(-participant) << std::endl;
    return 1;
  }

  dds_entity_t topic =
      dds_create_topic(participant, &geometry_msgs_msg_dds__Twist__desc,
                       "rt/cmd_vel", nullptr, nullptr);
  if (topic < 0) {
    std::cerr << "Failed to create topic: " << dds_strretcode(-topic)
              << std::endl;
    dds_delete(participant);
    return 1;
  }

  dds_qos_t *reader_qos = dds_create_qos();
  dds_qset_reliability(reader_qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
  dds_qset_history(reader_qos, DDS_HISTORY_KEEP_LAST, 10);

  dds_entity_t reader =
      dds_create_reader(participant, topic, reader_qos, nullptr);
  dds_delete_qos(reader_qos);

  if (reader < 0) {
    std::cerr << "Failed to create reader: "
              << dds_strretcode(-reader) << std::endl;
    dds_delete(participant);
    return 1;
  }

  std::cout << "Subscribed to DDS topic: rt/cmd_vel" << std::endl;

  while (true) {
    void *samples[1] = {nullptr};
    dds_sample_info_t infos[1];

    int rc = dds_take(reader, samples, infos, 1, 1);
    if (rc < 0) {
      std::cerr << "dds_take failed: " << dds_strretcode(-rc) << std::endl;
      break;
    }

    if (rc > 0 && infos[0].valid_data && samples[0] != nullptr) {
      auto *msg = static_cast<geometry_msgs_msg_dds__Twist_ *>(samples[0]);

      std::cout << "lin_x=" << msg->linear.x << " "
                << "lin_y=" << msg->linear.y << " "
                << "ang_z=" << msg->angular.z << std::endl;

      geometry_msgs_msg_dds__Twist__free(samples[0], DDS_FREE_ALL);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  dds_delete(participant);
  return 0;
}