var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
window.addEventListener('load', onload);

function onload(event)
{
    initWebSocket();
    initButton();
}

function initWebSocket()
{
    console.log('Trying to open a WebSockeet connection. . . ');
    websocket= new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event)
{
    console.log('Connection opened');
}

function onClose(event)
{
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function onMessage(event)
{
    if ((event.data == "ON") || (event.data == "OFF"))
    {
        document.getElementById('state').innerHTML = event.data;
    }    
    else
    {
        document.getElementById('battery').innerHTML = event.data;
    }    
    console.log(event.data);
}

function initButton()
{
    document.getElementById('bON').addEventListener('click', toggleON);
    document.getElementById('bOFF').addEventListener('click', toggleOFF);
}

function toggleON(event)
{
    websocket.send('bON');
}

function toggleOFF(event)
{
    websocket.send('bOFF');
}