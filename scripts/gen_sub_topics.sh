#!/bin/sh

echo "name: topic_monitor"
echo "topics:"
for i in `seq 1 50`;do
  i_0pad=`printf %02d $i`
  echo  "  - name: \"topic_monitor_topic${i_0pad}\""
  echo  "    topic_name: \"/topic${i_0pad}\""
  echo  "    acceptable_delay_ms: 500"
  echo  "    interval_ms: 100"
done
