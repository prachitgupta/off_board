#Mission 1 : Navigate to PRP via waypoints
		mav.wpPush(wps11)
		mav.set_mode("AUTO")
		#Take a note of what the reached index is prior to starting the mission
		n_prev=mav.reached_index
		#Set waypoint count to 0
		count = 0
		#Keep the script running till the first mission is completed
		while True:
			#Increment count by 1 every time there is a change in the index, ie a new waypoint is reached
			if n_prev!=mav.reached_index:
				count+=1 
				n_prev = mav.reached_index
			#End loop once the PRP is reached
			if count >= 17:
				break
			time.sleep(1)
		mav.wpClear()
		print("Out of the loop")

		mav.wpClear()
		#Put a 15 second wait for drop
		time.sleep(15)
		#Set to guided mode to reset parameters, start the next mission
		mav.set_Guided_mode()
		# Check
		print(mav.reached_index, ' After drop')
		#Mission 2: Return to launch point and fly over it to start the endurance run
		mav.wpPush(wps22)
		mav.set_mode("AUTO")
		count=0
		n_prev=mav.reached_index
		while True:
			print('Endurance run in progress')
			if time.time()>=warn_time or mav.bat_percentage<=30:
				mav.set_Guided_mode()
				mav.wpPush(wps3)
				mav.set_mode("AUTO")
				break
			if n_prev!=mav.reached_index:
				count+=1 
				n_prev = mav.reached_index
				print(n_prev)
			if count >=33 :
				break
			time.sleep(1)
		'''#Second run of the circuit
		if time.time()<warn_time:
			mav.wpPush(wps1)
			mav.set_mode("AUTO")
			count=0
			n_prev=mav.reached_index
			while True:
				if time.time()>=warn_time:
					mav.set_Guided_mode()
					mav.wpPush(wps3)
					mav.set_mode("AUTO")
					break
				if n_prev!=mav.reached_index:
					count+=1 
					n_prev = mav.reached_index
					print(n_prev)
				if count == 16:
					break
				time.sleep(1)'''
		# mav.land(5)	
		mav.toggle_arm(0) 