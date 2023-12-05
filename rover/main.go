package main

import (
	"fmt"
	"math"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/bluenviron/gomavlib/v2"
	"github.com/bluenviron/gomavlib/v2/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v2/pkg/dialects/common"
	"github.com/luisvinicius167/godux"
	"github.com/raspberrypi-go-drivers/hcsr04"
	"github.com/stianeikeland/go-rpio/v4"
)

/*

Librerias

Mavlink https://pkg.go.dev/github.com/aler9/gomavlib
GPIO https://pkg.go.dev/periph.io/x/conn/v3@v3.6.8/gpio
Godux //https:github.com/luisvinicius167/godux

*/

// var start time.Time

func main() {

	// Configuraciones iniciales
	mavlinkNode, err := configPixhawk()
	if err != nil {
		panic(err)
	}
	defer mavlinkNode.Close()

	// Apertura de puertos GPIO
	if err := rpio.Open(); err != nil {
		panic(err)
	}
	defer rpio.Close()

	sensors, err := configSensors()
	if err != nil {
		panic(err)
	}

	chanSensor := make(chan bool, 1)
	chanPX4Sensado := make(chan bool, 1) //sensado en px4

	chanPX4Comandos := make(chan bool, 1) //comandos desde raspi a px4

	// Inicio de rutinas
	go goduxApp(chanSensor, chanPX4Sensado, chanPX4Comandos)
	go runPixhawkSensor(mavlinkNode, chanPX4Sensado)
	go runPixhawkActuador(mavlinkNode, chanPX4Comandos)
	// for _, sensor := range sensors {
	// 	go runSensor(sensor, chanSensor)
	// }
	go runSensor(sensors[0], chanSensor)

	moveForward(mavlinkNode)

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGINT)
	<-c
	fmt.Println("\nFinalizando programa.")
	stopMotors(mavlinkNode)
	time.Sleep(100 * time.Millisecond)
	os.Exit(0)
}

// Configuración del PX4
func configPixhawk() (*gomavlib.Node, error) {
	fmt.Println("Configurando pixhawk...")
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointSerial{
				Device: "/dev/ttyUSB0",
				Baud:   57600,
			},
		},
		Dialect:     ardupilotmega.Dialect,
		OutVersion:  gomavlib.V2, // Cambiar a V1 si es imposible comunicarse con el objetivo
		OutSystemID: 10,
	})
	return node, err
}

// Configuración del Sensor
// func configSensors() (gpio.PinIn, error) {
func configSensors() ([]*hcsr04.HCSR04, error) {
	var sensors []*hcsr04.HCSR04
	sensors = append(sensors, hcsr04.NewHCSR04(3, 2))
	// sensors = append(sensors, hcsr04.NewHCSR04(17, 4))
	// sensors = append(sensors, hcsr04.NewHCSR04(22, 27))
	// sensors = append(sensors, hcsr04.NewHCSR04(23, 18))
	// sensors = append(sensors, hcsr04.NewHCSR04(25, 24))
	return sensors, nil
}

// Rutina de ejecución del Middleware Pixhawk
func runPixhawkSensor(node *gomavlib.Node, chanPX4 chan bool) {

	for evt := range node.Events() {
		if frm, ok := evt.(*gomavlib.EventFrame); ok {

			// fmt.Printf("received: id=%d, %+v\n", frm.Message().GetID(), frm.Message())
			if frm.Message().GetID() == 30 {

				switch msg := frm.Message().(type) {

				// if frm.Message() is a *ardupilotmega.MessageAttitude, access its fields
				case *ardupilotmega.MessageAttitude:
					// fmt.Println("received attitude output")
					// fmt.Println("TIME_BOOT_SEG", msg.TimeBootMs/1000, "s")
					// fmt.Println("ROLL", msg.Roll*(180/math.Pi), "°")
					// fmt.Println("PITCH", msg.Pitch*(180/math.Pi), "°")
					// fmt.Println("YAW", msg.Yaw*(180/math.Pi), "°")
					// fmt.Println("ROLLSPEED", msg.Rollspeed, "rad/s")
					// fmt.Println("PITCHSPEED", msg.Pitchspeed, "rad/s")
					// fmt.Println("YAWSPEED", msg.Yawspeed, "rad/s")
					// fmt.Println("")

					if msg.Yaw*(180/math.Pi) > 40 {
						// start = time.Now()

						chanPX4 <- true

						// node.WriteMessageTo(frm.Channel, &ardupilotmega.MessageParamValue{
						// 	ParamId:    "test_parameter",
						// 	ParamValue: 123456,
						// 	ParamType:  ardupilotmega.MAV_PARAM_TYPE_UINT32,
						// })

					}
				}
			}
		}
	}
}

// Rutina de ejecución del Middleware Actuador Pixhawk
func runPixhawkActuador(node *gomavlib.Node, chanPX4Comandos <-chan bool) {
	for {
		msg := <-chanPX4Comandos
		fmt.Println("Llega dato por el canal al actuador:", msg)
		if msg {
			fmt.Println("PONER EN MOVIMIENTO")
			// fmt.Println(time.Since(start))
			moveForward(node)
		} else {
			fmt.Println("GIRAR O DETENER")
			stopMotors(node)
		}
	}
}

// Rutina de ejecución del Middleware Sensor de Proximidad
// func runSensor(pin gpio.PinIn, chanSensor chan bool) {
func runSensor(sensor *hcsr04.HCSR04, chanSensor chan bool) error {

	sensor.StartDistanceMonitor()

	for {
		time.Sleep(hcsr04.MonitorUpdate)

		distance := sensor.GetDistance()

		if distance <= 0 {
			// fmt.Println("No se pudo medir la distancia, objeto muy lejano.")
			continue
		}
		fmt.Println("Distancia:", distance*100)
		if (distance * 100) < 20 {
			fmt.Println("DETECTO")
			chanSensor <- true
		} else {
			chanSensor <- false
			fmt.Println("NO DETECTO")
		}
	}

	// for {
	// 	pin.WaitForEdge(-1)
	// 	if pin.Read() == gpio.Low {
	// 		// fmt.Println("DETECTO")
	// 		chanSensor <- true
	// 	}
	// 	// else if pin.Read() == gpio.High {
	// 	// 	fmt.Println("NO DETECTO")
	// 	// }
	// }
}

// Maneja el estado de la aplicación
func goduxApp(chanSensor chan bool, chanPX4 chan bool, chanPX4Comandos chan bool) {

	store := godux.NewStore()       // Crea un nuevo store
	store.SetState("Running", true) // Setea el estado Running

	// acciones
	run := func() godux.Action {
		return godux.Action{
			Type:  "UPDATE_STATUS",
			Value: true,
		}
	}

	stop := func() godux.Action {
		return godux.Action{
			Type:  "UPDATE_STATUS",
			Value: false,
		}
	}

	reducer := func(action godux.Action) interface{} {
		switch action.Type {
		case "UPDATE_STATUS":
			return action.Value.(bool)

		default:
			return store.GetAllState()
		}
	}

	// Agrego la funcion reducer para devolver nuevos valores basados en el estado
	store.Reducer(reducer)

	for { // Quedo a la espera de mensajes provenientes de los canales
		select {
		case sensorValue := <-chanSensor:
			fmt.Println("------------ LLEGÓ ALGO A CHAN SENSOR:  ", sensorValue)
			// fmt.Println("Estado actual, Running:", store.GetState("Running"))
			if !sensorValue {
				fmt.Println("------------ CASO OMISO A CHAN SENSOR porque es false")
				continue
			}
			if store.GetState("Running") == true { // Si está en movimiento y llega la alerta Sensor, lo detengo
				newStatus := store.Dispatch(stop())
				store.SetState("Running", newStatus)
				fmt.Println("ALERTA Sensor!!!, Vehículo en movimiento: ", newStatus.(bool))

				// DETENER O GIRAR ENVIANDO COMANDO
				fmt.Println("Enviando false por canal chanPX4Comandos")
				chanPX4Comandos <- false

			} else {
				fmt.Println("LLegó la alerta Sensor, pero el vehículo ya se encuentra detenido")
			}

		case <-chanPX4:
			// fmt.Println("Estado actual, Running:", store.GetState("Running"))

			if store.GetState("Running") == false { // Si está detenido y llega la alerta PX4, lo pongo en movimiento
				newStatus := store.Dispatch(run())
				store.SetState("Running", newStatus)
				fmt.Println("ALERTA PX4!!!, Vehículo en movimiento: ", newStatus.(bool))

				// PONER EN MOVIMIENTO ENVIANDO COMANDO
				fmt.Println("Enviando true por canal chanPX4Comandos")
				chanPX4Comandos <- true

			} else {
				fmt.Println("LLegó la alerta PX4, pero el vehículo ya se encuentra en movimiento")
			}
		}
	}

}

func moveForward(node *gomavlib.Node) {
	fmt.Println("Por escribir mensaje para mover hacia adelante")
	writeMessage(node, 1800)
}

// func moveBackward(node *gomavlib.Node) {
// 	fmt.Println("Por escribir mensaje para mover hacia adelante")
// 	writeMessage(node, 1600)
// }

func stopMotors(node *gomavlib.Node) {
	fmt.Println("Por escribir mensaje para detener")
	writeMessage(node, 1700)
}

func writeMessage(node *gomavlib.Node, pulse float32) {
	fmt.Println("Escribiendo pulso", pulse)
	node.WriteMessageAll(&ardupilotmega.MessageCommandLong{
		Param1:          1,     // Servo index
		Param2:          pulse, // Servo position
		Command:         common.MAV_CMD(ardupilotmega.MAV_CMD_DO_SET_SERVO),
		TargetSystem:    0, // System ID
		TargetComponent: 0, // Component ID
		Confirmation:    0, // No confirmation
	})
	node.WriteMessageAll(&ardupilotmega.MessageCommandLong{
		Param1:          3,     // Servo index
		Param2:          pulse, // Servo position
		Command:         common.MAV_CMD(ardupilotmega.MAV_CMD_DO_SET_SERVO),
		TargetSystem:    0, // System ID
		TargetComponent: 0, // Component ID
		Confirmation:    0, // No confirmation
	})
}
