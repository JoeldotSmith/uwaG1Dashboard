class SubscriptionManager {
  constructor() {
    // {id: "id of wrapper" subscriptions: [subscription objects]}
    this.group_subscriptions = [];
  }

  get(wrapperid) {
    let group = this.group_subscriptions.find(group => group.id === wrapperid)
    if (group === undefined) return [];
    return group.subscriptions;
  }

  getAll() {
    return this.group_subscriptions;
  }

  subscribe(wrapperid, subscription) {
    let group = this.group_subscriptions.find(group => group.id === wrapperid)
    if (group === undefined) {
      group = {
        id: wrapperid,
        subscriptions: []
      }
      this.group_subscriptions.push(group);
    }
    group.subscriptions.push(subscription);
  };

  unsubcribe(wrapperid) {
    let groupIndex = this.group_subscriptions.findIndex(group => group.id === wrapperid)
    if (groupIndex === -1) return;

    this.group_subscriptions[groupIndex]
      .subscriptions
      .forEach(sub => {
        try {
          sub.unsubscribe();
        } catch (error) {
          console.warn(error);
        }
      });
    this.group_subscriptions.splice(groupIndex, 1);
  };

  unsubcribeAll() {
    this.group_subscriptions.forEach(group => {
      group.subscriptions.forEach(sub => {
        try {
          sub.unsubscribe();
        } catch (error) {
          console.warn(error);
        }
      });
    });
    this.group_subscriptions = [];
  }
}
