import paho.mqtt.client as paho

def on_message(mosq, obj, msg):
    if msg.topic == "wasp/state":
        print(msg.topic)
        payload = []
        for i in range(0, len(msg.payload)//2):
            payload.append(int.from_bytes(msg.payload[i*2:i*2+2], byteorder='big'))
        print(payload)
    elif msg.topic == "wasp/err":
        print(msg.topic)
        payload = []
        for i in range(0, len(msg.payload)):
            payload.append(int(msg.payload[i]))
        print(payload)
    else:
        print(msg.topic, msg.payload)
    #mosq.publish('pong', 'ack', 0)

def on_publish(*args):
    pass

if __name__ == '__main__':
    client = paho.Client(callback_api_version=paho.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.on_publish = on_publish
    client.username_pw_set("roboMQTT", "FIM4AMisgreat")

    #client.tls_set('root.ca', certfile='c1.crt', keyfile='c1.key')
    client.connect("10.152.213.31", 1883, 60)
    client.subscribe("wasp/err", 0)
    client.subscribe("wasp/state", 0)
    client.subscribe("wasp/log", 0)
    client.subscribe("wasp/config", 0)


    while client.loop() == 0:
        pass
