# Contact definition
uint8 STATE_OPEN=0
uint8 STATE_CLOSED=1
uint8 STATE_SLIPPING=2

Header header
string name
uint8 state
geometry_msgs/Wrench wrench
geometry_msgs/Point position
geometry_msgs/Vector3 normal
float64 frictionCoefficient
float64 restitutionCoefficient