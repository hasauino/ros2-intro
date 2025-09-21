const rclnodejs = require('rclnodejs');

async function createPublisher() {
    await rclnodejs.init();
    const node = new rclnodejs.Node('my_js_node');

    // Create a publisher for String messages on 'my_topic'
    const publisher = node.createPublisher('std_msgs/msg/String', 'my_topic');

    setInterval(() => {
        const message = "hello there from JavaScript!";
        publisher.publish(message);
    }, 1000);

    node.spin();
}

createPublisher().catch(console.error);