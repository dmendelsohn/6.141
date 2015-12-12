package MotionPlanning;

import org.ros.message.MessageListener;
import org.ros.message.lab5_msgs.GUIEraseMsg;

import Challenge.SonarGUI;

public class EraseMessageListener extends Challenge.EraseMessageListener
		implements MessageListener<GUIEraseMsg> {

	MapGUI gui;
	
	public EraseMessageListener(MapGUI mapGUI) {
		super(mapGUI);
		this.gui = mapGUI;
	}

	@Override
	public void onNewMessage(GUIEraseMsg arg0) {
		gui.eraseRects();
		gui.erasePolys();
		super.onNewMessage(arg0);
	}

}
