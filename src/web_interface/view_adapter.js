/*
 * Javascript that controls the meal view
 */

window_height = window.innerHeight;
window_width = window.innerWidth;
console.log("Window Height: " + window_height + ", Width: " + window_width);

var canvas = document.querySelector("#meal_view");
canvas.height = window_height;
canvas.width= window_width;
var ctx = canvas.getContext("2d");
ctx.fillStyle = "black";

var fitted_morsels

function new_morsel_handler(morsels) {
    scaled_morsels = fit_morsels_to_canvas(morsels, window_width, window_height, 50, 60);

    // flip the y coordinate
    for (var i = 0; i < scaled_morsels.length; i++) {
        scaled_morsels[i].y = window_height - scaled_morsels[i].y;
    }
    fitted_morsels = scaled_morsels;

    clear_canvas(window_width, window_height);
    draw_morsel(fitted_morsels);
}

// click listener
function get_morsel_at_position(morsels, hit_x, hit_y) {
    for(var i = 0; i < morsels.length; i++) {
        var x = morsels[i].x;
        var y = morsels[i].y;
        var r = morsels[i].r;

        if(Math.abs(x - hit_x) < r && Math.abs(y - hit_y) < r) {
            return i;
        }
    }
    // did not click on any morsel
    return -1;
}

function click_handler(hit_x, hit_y) {
    var morsel_index = get_morsel_at_position(fitted_morsels, hit_x, hit_y);
    if(morsel_index >= 0) {
        console.log("Hit morsel " + morsel_index);
        send_click_response(morsel_index);
    }
}

canvas.onclick = function(e) {
    var hit_x = e.clientX;
    var hit_y = e.clientY;
    // console.log("Click detected at: (" + hit_x + "," + hit_y + ")");
    click_handler(hit_x, hit_y);
};

// on screen indicator text
var connection_indicator = document.querySelector("#connection_indicator");
function update_connection_status(connected) {
    if(connected) {
        connection_indicator.className = "good";
        connection_indicator.innerHTML = "Connected";
    } else {
        connection_indicator.className = "bad";
        connection_indicator.innerHTML = "Connection Lost";
    }
}