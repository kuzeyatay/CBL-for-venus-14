import mqtt from 'mqtt'
import { updateRobot, addCellContent } from './canvas.js'

const robot1 = mqtt.connect('ws://localhost:9001', {
  username: 'robot_81_1',
  password: 'hZyDS0OF'
})
const robot2 = mqtt.connect('ws://localhost:9001', {
  username: 'robot_83_1',
  password: 'GsPO5eY7'
})

const r1_send = '/pynqbridge/81/send'
const r1_recieve = '/pynqbridge/81/recv'
const r2_send = '/pynqbridge/83/send'
const r2_recieve = '/pynqbridge/83/recv'

function sendMessage(client, topic, message) {
  client.publish(topic, message)
  console.log(`Published: ${message}`)
}

function parseMessage(message) {
  return JSON.parse(message)
}


// json shape:
// {
//   robotId: number,          // e.g. 81
//   col:     number,          // robot cell column
//   row:     number,          // robot cell row
//   heading: number,          // radians
//   rock?:  { col, row, color?, size?, temperature? },
//   cliff?: { col, row },
//   hill?:  { col, row }
// }

export function updateCanvasFromRobotData(parsedJson) {
  const { robotId, col, row, heading, rock, cliff, hill } = parsedJson

  updateRobot(robotId, col, row, heading)

  if (rock) {
    addCellContent(rock.col, rock.row, {
      type: 'rock',
      color: rock.color,
      size: rock.size,
      temperature: rock.temperature,
    })
  }

  if (cliff) {
    addCellContent(cliff.col, cliff.row, { type: 'cliff' })
  }

  if (hill) {
    addCellContent(hill.col, hill.row, { type: 'hill' })
  }
}

// test robots moving in circle
async function test(robot, topic, id) {
  const centerCol = 0
  const centerRow = 0
  const radius = 4 

  for (let t = 0; t < Math.PI * 2; t += 0.15) {
    const col = Math.round(centerCol + radius * Math.cos(t))
    const row = Math.round(centerRow + radius * Math.sin(t))
    const heading = -t - Math.PI / 2

    const rockCol   = col + Math.round(Math.cos(heading))
    const rockRow   = row - Math.round(Math.sin(heading))
    const cliffCol  = col + Math.round(2 * Math.cos(heading))
    const cliffRow  = row - Math.round(2 * Math.sin(heading))
    const hillCol   = col + Math.round(3 * Math.cos(heading))
    const hillRow   = row - Math.round(3 * Math.sin(heading))

    sendMessage(robot, topic, JSON.stringify({
      robotId: id,
      col,
      row,
      heading,
      rock:  { col: rockCol,  row: rockRow,  color: 'black', size: 3, temperature: 22 },
      cliff: { col: cliffCol, row: cliffRow },
      hill:  { col: hillCol,  row: hillRow  },
    }))

    await new Promise(r => setTimeout(r, 120))
  }
}

// event handlers
robot1.on('connect', () => {
  console.log('Connected to Robot 1')
  robot1.subscribe(r1_send, err => {
    if (!err) test(robot1, r1_send, 81)
  })
})

robot2.on('connect', () => {
  console.log('Connected to Robot 2')
  robot2.subscribe(r2_send, err => {
    if (!err) test(robot2, r2_send, 83)
  })
})

robot1.on('message', (topic, message) => {
  const data = parseMessage(message.toString())
  console.log('Robot 1 recv:', topic, data)
  updateCanvasFromRobotData(data)
})

robot2.on('message', (topic, message) => {
  const data = parseMessage(message.toString())
  console.log('Robot 2 recv:', topic, data)
  updateCanvasFromRobotData(data)
})
