/*
License: BSD
https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
*/

// @flow
import rosnodejs from 'rosnodejs';
//TODO: implement

class PouchWorldState {
  nh: Object;
  chatterPub: Object;

  constructor(nh) {
    this.nh = nh;
    this.initPublishers();
  }

  initPublishers = () => {
    this.chatterPub = this.nh.advertise('/chatter', 'std_msgs/String');
  };

  publish = () => {
    this.chatterPub.publish({ data: "hi" });
  };

}

rosnodejs.initNode('/world_state')
  .then(() => {
    const nh = rosnodejs.nh;
    const pouchWorldState = new PouchWorldState(nh);

    setInterval(pouchWorldState.publish, 100);
});
