/* Robot identity is fixed for a document lifetime. Switching reloads all caches. */
(()=>{'use strict';
 const params=new URLSearchParams(location.search);let saved=null;try{saved=localStorage.getItem('fleet.robot');}catch(_){}
 const robotId=params.get('robot')||saved||'go2w-01';
 function url(value){const u=new URL(value,location.href);if(u.origin===location.origin&&u.pathname.startsWith('/api/')&&!u.pathname.startsWith('/api/fleet/')&&!u.pathname.startsWith('/api/robots/'))u.pathname='/api/robots/'+encodeURIComponent(robotId)+'/'+u.pathname.slice(5);return u.href;}
 const nativeFetch=window.fetch.bind(window);
 window.fetch=(input,options)=>nativeFetch(input instanceof Request?new Request(url(input.url),input):url(String(input)),options);
 const descriptor=Object.getOwnPropertyDescriptor(HTMLImageElement.prototype,'src');
 Object.defineProperty(HTMLImageElement.prototype,'src',{...descriptor,set(value){descriptor.set.call(this,value?url(String(value)):value);}});
 window.FleetContext={robotId,url,nativeFetch,select(id){const u=new URL(location.href);u.searchParams.set('robot',id);const tab=document.querySelector('.tab.active')?.dataset.tab;if(tab)u.searchParams.set('tab',tab);try{localStorage.setItem('fleet.robot',id);}catch(_){}location.assign(u);}};
})();
