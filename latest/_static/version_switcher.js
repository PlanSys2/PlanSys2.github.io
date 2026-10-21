// Copyright 2026 Intelligent Robotics Lab
// Version selector and banner: the site is /<version>/..., the versions are in /versions.json
(function () {
  'use strict';

  var parts = window.location.pathname.split('/');  // ['', '<version>', ...page]
  var current = parts[1];
  var page = parts.slice(2).join('/');

  // Opens the same page in another version, or its home page if it does not exist there
  function go(version) {
    var target = '/' + version + '/' + page;
    fetch(target, {method: 'HEAD'}).then(function (r) {
      window.location.href = (r.ok ? target : '/' + version + '/') + window.location.hash;
    }).catch(function () {
      window.location.href = '/' + version + '/';
    });
  }

  function banner(text, version, title) {
    var body = document.querySelector('div[itemprop="articleBody"]');
    if (!body) {return;}
    var note = document.createElement('div');
    note.className = 'admonition warning';
    var p = document.createElement('p');
    p.appendChild(document.createTextNode(text + ' '));
    var link = document.createElement('a');
    link.href = '#';
    link.textContent = 'Go to ' + title + '.';
    link.addEventListener('click', function (e) {e.preventDefault(); go(version);});
    p.appendChild(link);
    note.appendChild(p);
    body.insertBefore(note, body.firstChild);
  }

  fetch('/versions.json').then(function (r) {return r.json();}).then(function (data) {
    var known = data.versions.map(function (v) {return v.name;});
    if (known.indexOf(current) < 0) {return;}  // Not a versioned page (e.g. a local build)

    var titles = {};
    data.versions.forEach(function (v) {titles[v.name] = v.title || v.name;});
    document.getElementById('plansys2-current-version').textContent = titles[current];

    var list = document.getElementById('plansys2-version-list');
    data.versions.forEach(function (v) {
      var dd = document.createElement('dd');
      var a = document.createElement('a');
      a.href = '/' + v.name + '/';
      a.textContent = v.title || v.name;
      if (v.name === current) {a.style.fontWeight = 'bold';}
      a.addEventListener('click', function (e) {e.preventDefault(); go(v.name);});
      dd.appendChild(a);
      list.appendChild(dd);
    });
    document.getElementById('plansys2-versions').style.display = '';

    var latestRelease = titles[data.default];
    if (current === 'latest') {
      banner('This is the development documentation: it may describe features not released yet.',
        data.default, 'the documentation of the latest release (' + latestRelease + ')');
    } else if (current !== data.default) {
      banner('This is the documentation of PlanSys2 ' + titles[current] +
        ', not the latest release (' + latestRelease + ').', data.default, latestRelease);
    }
  }).catch(function () {});
})();
