// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org


/* 
package frc.robot.util;
import edu.wpi.first.wpilibj.DriverStation;
import fi.iki.elonen.NanoHTTPD;

public class WebServer extends NanoHTTPD {


    // This is a test web server class i created, hopefully it can be used on future robots!
    // It uses the NanoTTPD libary to create a webserver on port 8080 and simply displays a String of html code, pretty simple :)

    public WebServer() {
        super(8080);
        
        DriverStation.reportWarning("RoboRio Webserver started at: " + super.getListeningPort(), true);
    }

    @Override
    public Response serve(IHTTPSession session) {

        // Top of page html
        String html = "<html><body>";

        html += "RoboRio Webserver Test" + "<br>";

        // Bottom of page html
        html += "</body></html>";

        return newFixedLengthResponse(Response.Status.OK, "text/html", html);
    }

}

*/