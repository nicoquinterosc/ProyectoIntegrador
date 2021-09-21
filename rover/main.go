package main

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"strings"

	"github.com/aler9/gomavlib"
	"github.com/aler9/gomavlib/pkg/dialects/ardupilotmega"
	"github.com/luisvinicius167/godux"
	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/gpio/gpioreg"
	"periph.io/x/host/v3"
)

/*

Librerias

Mavlink https://pkg.go.dev/github.com/aler9/gomavlib@v0.0.0-20210830132230-707edfd2f139
GPIO https://pkg.go.dev/periph.io/x/conn/v3@v3.6.8/gpio
Godux //https:github.com/luisvinicius167/godux

*/

func main() {

	// Configuraciones iniciales
	mavlinkNode, errML := configPixhawk()

	if errML != nil {
		panic(errML)
	}
	defer mavlinkNode.Close()

	pin, errS := configSensor()

	if errS != nil {
		panic(errS)
	}

	chanSensor := make(chan bool, 1)
	chanPX4 := make(chan bool, 1)

	// Inicio de rutinas
	go goduxApp(chanSensor, chanPX4)
	go runPixhawk(mavlinkNode, chanPX4)
	go runSensor(pin, chanSensor)

	for {
		reader := bufio.NewReader(os.Stdin)
		text, _ := reader.ReadString('\n')
		text = strings.Replace(text, "\n", "", -1)
		if strings.Compare("exit", text) == 0 {
			close(chanSensor)
		}
		continue
	}
}

// Configuración del PX4
func configPixhawk() (*gomavlib.Node, error) {
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointSerial{Address: "/dev/ttyUSB0:57600"},
		},
		Dialect:     ardupilotmega.Dialect,
		OutVersion:  gomavlib.V2, // Cambiar a V1 si es imposible comunicarse con el objetivo
		OutSystemID: 10,
	})

	return node, err
}

// Configuración del Sensor
func configSensor() (gpio.PinIn, error) {

	if _, err := host.Init(); err != nil { //Carga los perifericos
		return nil, err
	}

	// Configura pin por nombre
	pin := gpioreg.ByName("18")

	// Seteo como input
	err := pin.In(gpio.PullDown, gpio.RisingEdge)

	return pin, err
}

// Rutina de ejecución del Middleware Pixhawk
func runPixhawk(node *gomavlib.Node, chanPX4 chan bool) {

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
						chanPX4 <- true
					}
				}
			}
		}
	}
}

// Rutina de ejecución del Middleware Sensor de Proximidad
func runSensor(pin gpio.PinIn, chanSensor chan bool) {

	for {
		pin.WaitForEdge(-1)
		if pin.Read() == gpio.Low {
			// fmt.Println("DETECTO")
			chanSensor <- true
		}
		// else if pin.Read() == gpio.High {
		// 	fmt.Println("NO DETECTO")
		// }
	}
}

// Maneja el estado de la aplicación
func goduxApp(chanSensor chan bool, chanPX4 chan bool) {

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
		case <-chanSensor:

			// fmt.Println("Estado actual, Running:", store.GetState("Running"))

			if store.GetState("Running") == true { // Si está en movimiento y llega la alerta Sensor, lo detengo
				newStatus := store.Dispatch(stop())
				store.SetState("Running", newStatus)
				fmt.Println("ALERTA Sensor!!!, Vehículo en movimiento: ", newStatus.(bool))
			} else {
				fmt.Println("LLegó la alerta Sensor, pero el vehículo ya se encuentra detenido")
			}

		case <-chanPX4:

			// fmt.Println("Estado actual, Running:", store.GetState("Running"))

			if store.GetState("Running") == false { // Si está detenido y llega la alerta PX4, lo pongo en movimiento
				newStatus := store.Dispatch(run())
				store.SetState("Running", newStatus)
				fmt.Println("ALERTA PX4!!!, Vehículo en movimiento: ", newStatus.(bool))
			} else {
				fmt.Println("LLegó la alerta PX4, pero el vehículo ya se encuentra en movimiento")
			}
		}
	}

}
