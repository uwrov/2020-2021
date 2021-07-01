
let WIDGET_COUNT = 0;

export class Widget {
  constructor(type) {
    this.id = WIDGET_COUNT++;
    this.type = type;
    this.savedProps = {
      saveProp: this.saveProp,
    };
  }

  saveProp = (object) => {
    this.savedProps = Object.assign(this.savedProps, object);
  };
}

export default Widget;
