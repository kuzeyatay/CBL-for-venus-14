// proxy.js - run with: node proxy.js
const net = require('net')
const WebSocket = require('ws')

const wss = new WebSocket.Server({ port: 9001 })

wss.on('connection', (ws) => {
  const tcp = net.createConnection(1883, 'mqtt.ics.ele.tue.nl')

  tcp.on('data', (data) => ws.send(data))
  ws.on('message', (data) => tcp.write(data))

  tcp.on('close', () => ws.close())
  ws.on('close', () => tcp.destroy())
})

console.log('Proxy running on ws://localhost:9001')
