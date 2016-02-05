/*
 * Javascript that draws the meal view
 */

function scale_morsels(morsels, x_offset, y_offset, scale) {
    // console.log("scaling with x_offset=" + x_offset + ", y_offset=" + y_offset + ", scale=" + scale);
    for(var i = 0; i < morsels.length; i++) {
        var x = morsels[i].x;
        var y = morsels[i].y;
        var r = morsels[i].r;

        var new_x = x*scale - x_offset;
        var new_y = y*scale - y_offset;
        var new_r = r*scale;

        // console.log("Scaling:\nx: " + x + " -> " + new_x +
        //                     "\ny: " + y + " -> " + new_y +
        //                     "\nr: " + r + " -> " + new_r);

        morsels[i].x = new_x;
        morsels[i].y = new_y;
        morsels[i].r = new_r;
    }

    return morsels;
}

function fit_morsels_to_canvas(morsels, canvas_width, canvas_height, padding, radius_limit) {
    if(morsels.length == 0) {
        return [];
    }

    canvas_height = canvas_height - 2*padding;
    canvas_width = canvas_width - 2*padding;
    console.log("fitting morsels to screen size");
    var max_x = morsels[0].x + morsels[0].r;
    var min_x = max_x;
    var max_y = morsels[0].y + morsels[0].r;
    var min_y = max_y;

    var max_r = morsels[0].r;

    for(var i = 0; i < morsels.length; i++) {
        var curr_x_max = morsels[i].x + morsels[i].r;
        var curr_x_min = morsels[i].x - morsels[i].r;
        var curr_y_max = morsels[i].y + morsels[i].r;
        var curr_y_min = morsels[i].y - morsels[i].r;

        if(curr_x_max > max_x) {
            max_x = curr_x_max;
        } else if(curr_x_min < min_x) {
            min_x = curr_x_min;
        }

        if(curr_y_max > max_y) {
            max_y = curr_y_max;
        } else if(curr_y_min < min_y) {
            min_y = curr_y_min;
        }

        if(morsels[i].r > max_r) {
            max_r = morsels[i].r;
        }
    }

    // console.log("min x: " + min_x +
    //           "\nmax_x: " + max_x +
    //           "\nmin_y: " + min_y +
    //           "\nmax_y: " + max_y);

    var max_scale = radius_limit/max_r;
    var scale = 1;
    var x_diff = max_x - min_x;
    var y_diff = max_y - min_y;

    if(x_diff/y_diff > canvas_width/canvas_height) {
        scale = canvas_width / x_diff;
    } else {
        scale = canvas_height / y_diff;
    }

    if(scale > max_scale) {
        scale = max_scale;
    }

    var x_offset = min_x*scale - padding;
    var y_offset = min_y*scale - padding;

    return scale_morsels(morsels, x_offset, y_offset, scale);
}

function draw_morsel(morsels) {
    console.log("Drawing new morsels");
    for (var i = 0; i < morsels.length; i++) {
        var bite = morsels[i];

        ctx.beginPath();
        ctx.arc(bite.x, bite.y, bite.r, 0, 2*Math.PI);
        ctx.fill();
    }
}

function highlight_morsel(bite) {
    ctx.fillStyle = "green";
    ctx.beginPath();
    ctx.arc(bite.x, bite.y, bite.r, 0, 2*Math.PI);
    ctx.fill();
    ctx.fillStyle = "black"

    setTimeout(function() {
        ctx.beginPath();
        ctx.arc(bite.x, bite.y, bite.r, 0, 2*Math.PI);
        ctx.fill();
    }, 200);
}

function clear_canvas(canvas_width, canvas_height) {
    ctx.clearRect(0, 0, canvas_width, canvas_height);
}