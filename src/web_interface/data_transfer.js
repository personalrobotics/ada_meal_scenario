/*
 * Javascript that fetches data from server and sends back click responses
 */

test_morsels = [{x:300, y:400, r:70},
           {x:500, y:800, r:85},
           {x:1000, y:900, r:60}];
test_morsels2 = [{x:100, y:100, r:50},
            {x:200, y:200, r:50}];

function handle_morsel_data(response) {
    response = response.replace(/'/g, '"');
    console.log(response);
    new_morsel_handler(JSON.parse(response));
}

function retrieve_morsel() {
    var get_morsel_request = new XMLHttpRequest();
    get_morsel_request.onreadystatechange = function () {
        if(get_morsel_request.readyState == 4 && get_morsel_request.status == 200) {
            console.log("server supplied data");
            update_connection_status(true);
            handle_morsel_data(get_morsel_request.responseText);
        } else {
            // console.log("error occurred retrieving data");
            update_connection_status(false);
        }
    };
    get_morsel_request.open('POST', "/serve_morsels", true);
    get_morsel_request.setRequestHeader("Content-type","application/x-www-form-urlencoded");
    get_morsel_request.send();
}

function send_click_response(index) {
    var params = "i=" + index;
    var send_click_request = new XMLHttpRequest();
    send_click_request.onreadystatechange = function() {
        if(send_click_request.readyState == 4 && send_click_request.status == 200) {
            console.log("server accepted response. Replied with: " + send_click_request.responseText);
            highlight_morsel(fitted_morsels[index]);
            update_connection_status(true);
        } else {
            // console.log("error occurred submitting response");
            update_connection_status(false);
        }
    };
    send_click_request.open('POST', "/register_click", true);
    send_click_request.setRequestHeader("Content-type","application/x-www-form-urlencoded");
    send_click_request.send(params);
}
