function get_data()
{
    var req = false;

    function getDataComplete()
    {
        if(req.readyState == 4)
        {
            if(req.status == 200)
            {
                document.getElementById("serverValue").innerHTML = req.responseText
            }
        }
    }

    if(window.XMLHttpRequest)
    {
        req = new XMLHttpRequest();
    }
    else if(window.ActiveXObject)
    {
        req = new ActiveXObject("Microsoft.XMLHTTP");
    }
    if(req)
    {
        req.open("GET", "/param?id=" + Math.random(), true);
        req.onreadystatechange = getDataComplete;
        req.send(null);
    }
}
