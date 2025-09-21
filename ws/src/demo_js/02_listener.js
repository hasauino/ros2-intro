const rclnodejs = require('rclnodejs');

async function createSubscriber() {
    await rclnodejs.init();
    const node = new rclnodejs.Node('listener_js');

    // Create a subscriber for String messages on 'my_topic'
    node.createSubscription('std_msgs/msg/String', 'my_topic', (msg) => {
        console.log(`I got this msg: ${typeof msg}`, msg);
    });

    node.spin();
}

createSubscriber().catch(console.error);