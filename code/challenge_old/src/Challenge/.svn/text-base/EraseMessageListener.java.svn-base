package Challenge;

import org.ros.message.MessageListener;
//import org.ros.message.challenge_msgs.GUIEraseMsg;
import org.ros.message.lab5_msgs.GUIEraseMsg;

public class EraseMessageListener implements MessageListener<GUIEraseMsg> {

	private SonarGUI gui;
	
	public EraseMessageListener(SonarGUI sonarGUI) {
		this.gui = sonarGUI;
	}

	@Override
	public void onNewMessage(GUIEraseMsg arg0) {
		gui.eraseLine();
		gui.erasePoints();
		gui.eraseSegments();
	}

}
