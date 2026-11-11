// Event timeline (VIE-002): marks from the facade's `timeline` entries at their simulated time; categories and
// labels come from the facade (VIE-014).

function clear(element) {
  while (element.firstChild) {
    element.removeChild(element.firstChild);
  }
}

export class Timeline {
  constructor(bar, list, onPick) {
    this.bar = bar;
    this.list = list;
    this.onPick = onPick;
  }

  // `entries` from `timeline`; `first` and `last` the replay's recorded time span in s.
  show(entries, first, last) {
    clear(this.bar);
    clear(this.list);
    for (const entry of entries) {
      const mark = document.createElement("button");
      mark.className = `mark ${entry.category}`;
      mark.title = `${entry.t} s  ${entry.category}: ${entry.label}`;
      let fraction = 0;
      if (last > first) {
        fraction = (entry.t - first) / (last - first);
      }
      mark.style.left = `${Math.min(Math.max(fraction, 0), 1) * 100}%`;
      mark.addEventListener("click", () => this.onPick(entry.t));
      this.bar.appendChild(mark);
      const row = document.createElement("li");
      row.className = entry.category;
      const time = document.createElement("button");
      time.textContent = `${entry.t} s`;
      time.addEventListener("click", () => this.onPick(entry.t));
      const label = document.createElement("span");
      label.textContent = `${entry.category}  ${entry.label}  (${entry.source})`;
      row.append(time, label);
      this.list.appendChild(row);
    }
    if (entries.length === 0) {
      const row = document.createElement("li");
      row.textContent = "no marked events";
      this.list.appendChild(row);
    }
  }
}
