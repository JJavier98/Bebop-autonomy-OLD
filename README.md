# TFG-Dron-de-Vigilancia
Sistema activo para protección de Infraestructuras Críticas:
El sistema pasivo de vigilancia detecta un potencial intruso/peligro en el entorno monitorizado de la infrastructura.
Obtiene posición 3D en el espacio y manda un drone (o flotilla) a esa posición en caso de que se pierda del campo de visión de la cámara.
El drone sigue al objetivo fuera del campo de visión de la cámara, mandando stream de vídeo al servidor y siguiendo al mismo (detección + tracking).
Aún no decidimos sobre si este procesamiento se realiza a bordo o en el servidor (diferentes objetivos, si es a bordo, el objetivo será más centrado en la arquitectura - prestaciones, tiempo de ejecución, autonomía, etc; si por el contrario, lo hacemos en el servidor, el peso irá más sobre el desarrollo que se lleve a cabo, estrategias para aumentar autonomía, comunicaciones, etc).
