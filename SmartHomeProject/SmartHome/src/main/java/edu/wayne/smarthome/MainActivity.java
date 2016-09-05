package edu.wayne.smarthome;

import android.app.Activity;
import android.os.Bundle;
import org.w3c.dom.Document;
import org.w3c.dom.NodeList;

public class MainActivity extends Activity {
    // All static variables
    //static final String URL = "192.168.1.1/leds.cgi?led=1";
    static final String URL = "192.168.1.1/status.xml";

    // XML node keys
    static final String KEY_ITEM = "item"; // parent node
    static final String KEY_NAME = "name";
    static final String KEY_COST = "cost";
    static final String KEY_DESC = "description";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        XMLParser parser = new XMLParser();
        String xml = parser.getXmlFromUrl(URL); // getting XML
        Document doc = parser.getDomElement(xml); // getting DOM element

        NodeList nl = doc.getElementsByTagName(KEY_ITEM);

// looping through all item nodes <item>
        for (int i = 0; i < nl.getLength(); i++) {
            String name = parser.getValue(e, KEY_NAME); // name child value
            String cost = parser.getValue(e, KEY_COST); // cost child value
            String description = parser.getValue(e, KEY_DESC); // description child value
        }

    }





}
