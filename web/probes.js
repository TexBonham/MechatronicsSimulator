// Probe panel (VIE-003 to VIE-006): columns from the facade's `probe`, one value per recorded time, shown with the
// unit each column names; a single reference read through `value_at` (FAC-011). Nothing is computed here (FAC-016).

function clear(element) {
  while (element.firstChild) {
    element.removeChild(element.firstChild);
  }
}

// Index of the latest recorded time at or before t (the first when t precedes it): a lookup, no arithmetic on values.
export function sampleAt(times, t) {
  let low = 0;
  let high = times.length - 1;
  if (high < 0) {
    return -1;
  }
  if (t < times[0]) {
    return 0;
  }
  while (low < high) {
    const middle = (low + high + 1) >> 1;
    if (times[middle] <= t) {
      low = middle;
    } else {
      high = middle - 1;
    }
  }
  return low;
}

export class ProbePanel {
  constructor(table, caption) {
    this.table = table;
    this.caption = caption;
    this.probe = null;
  }

  // `probe` is the `probe` result (times, columns of name, unit, values).
  set(probe) {
    this.probe = probe;
    this.caption.textContent = `${probe.kind} ${probe.name}`;
  }

  clearProbe() {
    this.probe = null;
    this.caption.textContent = "";
    clear(this.table);
  }

  // Shows the probe's recorded values at the cursor time `t` (s).
  show(t) {
    clear(this.table);
    if (this.probe === null) {
      return;
    }
    const index = sampleAt(this.probe.times, t);
    if (index < 0) {
      return;
    }
    const head = document.createElement("tr");
    for (const text of ["t", `${this.probe.times[index]} s`]) {
      const cell = document.createElement("th");
      cell.textContent = text;
      head.appendChild(cell);
    }
    this.table.appendChild(head);
    for (const column of this.probe.columns) {
      const row = document.createElement("tr");
      const name = document.createElement("td");
      name.textContent = column.name;
      const value = document.createElement("td");
      value.textContent = `${column.values[index]} ${column.unit}`;
      row.append(name, value);
      this.table.appendChild(row);
    }
  }
}
